"""One SocketCAN-FD session per interface, shared by the arm and gripper."""

from __future__ import annotations

import logging
import socket
import struct
import threading
import time
from dataclasses import dataclass, field

from openarm_viam.can.damiao import (
    CLEAR_ERROR,
    DISABLE,
    ENABLE,
    MODE_POS_FORCE,
    STATUS_ENABLED,
    command_packet,
    control_mode_packet,
    mit_packet,
    parse_state,
    posforce_packet,
    refresh_packet,
)

LOGGER = logging.getLogger(__name__)
CONTROL_PERIOD_S = 0.01
CONFIGURE_CYCLES = 3
ENABLE_CYCLES = 5
CANFD_MTU = 72
CAN_MTU = 16
CANFD_BRS = 0x01
CAN_ERR_FLAG = 0x20000000
_CAN_ERRORS = (
    (0x00000001, "tx-timeout"),
    (0x00000004, "controller"),
    (0x00000020, "ack"),
    (0x00000040, "bus-off"),
    (0x00000100, "restarted"),
)


@dataclass
class Motor:
    motor_type: str
    send_id: int
    recv_id: int
    mode: str = "refresh"
    control: str = "mit"
    enable_cycles: int = 0
    configure_cycles: int = 0
    has_target: bool = False
    target: float = 0.0
    kp: float = 0.0
    kd: float = 0.0
    speed: float = 0.0
    torque_pu: float = 0.0
    measured: float | None = None
    velocity: float = 0.0
    torque: float = 0.0
    error: int | None = None


def commands_for(motor: Motor) -> list[tuple[int, bytes]]:
    """Next frames for one motor.

    A latched fault (for example communication lost) makes the motor ignore
    enable while it still answers status requests, so faults are cleared
    first. Only the position-force gripper gets a control-mode write, as in
    the OpenArm driver. Enable repeats until the motor reports it is enabled;
    position commands start from the measured angle.
    """

    if motor.mode == "configure":
        motor.configure_cycles += 1
        if motor.configure_cycles >= CONFIGURE_CYCLES:
            motor.mode = "enable"
            motor.enable_cycles = 0
        frames = [(motor.send_id, command_packet(CLEAR_ERROR))]
        if motor.control == "posforce":
            frames.append(control_mode_packet(motor.send_id, MODE_POS_FORCE))
        return frames
    if motor.mode == "enable":
        motor.enable_cycles += 1
        ready = motor.enable_cycles >= ENABLE_CYCLES and motor.measured is not None
        if ready and motor.error == STATUS_ENABLED:
            if not motor.has_target:
                motor.target = motor.measured
            motor.mode = "mit"
        elif motor.enable_cycles % 20 == 0:
            return [(motor.send_id, command_packet(CLEAR_ERROR))]
        return [(motor.send_id, command_packet(ENABLE))]
    if motor.mode == "mit":
        if motor.control == "posforce":
            return [(motor.send_id + 0x300, posforce_packet(motor.target, motor.speed, motor.torque_pu))]
        return [(motor.send_id, mit_packet(motor.motor_type, motor.target, 0.0, motor.kp, motor.kd, 0.0))]
    if motor.mode == "disable":
        motor.mode = "idle"
        return [(motor.send_id, command_packet(DISABLE))]
    if motor.mode == "refresh":
        return [refresh_packet(motor.send_id)]
    return []


def pack_canfd(can_id: int, data: bytes) -> bytes:
    """CAN-FD frame with bitrate switch, and no CANFD_FDF flag.

    The kernel sets CANFD_FDF on frames it delivers. Putting that flag on a
    transmit buffer makes Linux return EINVAL, so the frame never reaches the
    wire. OpenArm motors then see silence and turn the LED red.
    """

    payload = bytes(data[:64]).ljust(64, b"\x00")
    return struct.pack("<IBBBB64s", can_id & 0x7FF, len(data), CANFD_BRS, 0, 0, payload)


def pack_classic(can_id: int, data: bytes) -> bytes:
    payload = bytes(data[:8]).ljust(8, b"\x00")
    return struct.pack("<IB3x8s", can_id & 0x7FF, min(len(data), 8), payload)


def interface_supports_fd(interface: str) -> bool:
    """An interface brought up with ``fd on`` has a 72-byte MTU; classic CAN has 16."""

    try:
        with open(f"/sys/class/net/{interface}/mtu", encoding="ascii") as handle:
            return int(handle.read().strip()) >= CANFD_MTU
    except (OSError, ValueError):
        return True


def unpack_frame(raw: bytes) -> tuple[int, bytes, bool] | None:
    if len(raw) not in (CAN_MTU, CANFD_MTU):
        return None
    can_id, length = struct.unpack_from("<IB", raw)
    length = min(length, len(raw) - 8)
    return can_id, raw[8 : 8 + length], bool(can_id & CAN_ERR_FLAG)


def bus_error_name(can_id: int) -> str:
    names = [name for bit, name in _CAN_ERRORS if can_id & bit]
    return ",".join(names) or hex(can_id)


def open_can_socket(interface: str) -> socket.socket:
    if not hasattr(socket, "PF_CAN"):
        raise RuntimeError("SocketCAN is only available on Linux")
    raw = socket.socket(socket.PF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
    level = getattr(socket, "SOL_CAN_RAW", 101)
    raw.setsockopt(level, getattr(socket, "CAN_RAW_FD_FRAMES", 5), 1)
    error_mask = 0
    for bit, _name in _CAN_ERRORS:
        error_mask |= bit
    raw.setsockopt(level, getattr(socket, "CAN_RAW_ERR_FILTER", 2), struct.pack("I", error_mask))
    raw.bind((interface,))
    raw.settimeout(0.002)
    return raw


@dataclass
class CanSession:
    interface: str
    motors: dict[int, Motor] = field(default_factory=dict)
    refs: int = 0
    _lock: threading.Lock = field(default_factory=threading.Lock)
    _stop: threading.Event = field(default_factory=threading.Event)
    _thread: threading.Thread | None = None
    _socket: socket.socket | None = None
    tx_ok: int = 0
    tx_errors: int = 0
    rx_ok: int = 0
    bus_errors: int = 0
    last_error: str = ""
    last_bus: str = ""
    rx_other: int = 0
    other_ids: set[int] = field(default_factory=set)
    fd: bool = True

    def start(self) -> None:
        self.fd = interface_supports_fd(self.interface)
        self._socket = open_can_socket(self.interface)
        self._thread = threading.Thread(target=self._loop, name=f"openarm-{self.interface}", daemon=True)
        self._thread.start()
        LOGGER.info("opened %s session on %s", "CAN-FD" if self.fd else "classic CAN", self.interface)

    def stop(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=1.0)
        raw = self._socket
        self._socket = None
        if raw is not None:
            raw.close()
        LOGGER.info("closed CAN-FD session on %s", self.interface)

    def add_motor(self, motor: Motor) -> None:
        with self._lock:
            if motor.send_id in self.motors:
                raise ValueError(f"CAN id 0x{motor.send_id:02x} is already used on {self.interface}")
            self.motors[motor.send_id] = motor

    def drop_motors(self, send_ids: list[int]) -> None:
        with self._lock:
            for send_id in send_ids:
                self.motors.pop(send_id, None)

    def measured(self, send_id: int) -> float | None:
        with self._lock:
            motor = self.motors.get(send_id)
            return None if motor is None else motor.measured

    def status(self, send_id: int) -> int | None:
        with self._lock:
            motor = self.motors.get(send_id)
            return None if motor is None else motor.error

    def commanding(self, send_ids: list[int]) -> bool:
        """True once every motor is enabled and receiving position commands."""

        with self._lock:
            return all(
                send_id in self.motors
                and self.motors[send_id].mode == "mit"
                and self.motors[send_id].error == STATUS_ENABLED
                for send_id in send_ids
            )

    def set_target(self, send_id: int, motor_position: float, kp: float, kd: float) -> None:
        with self._lock:
            motor = self.motors[send_id]
            motor.target = motor_position
            motor.kp = kp
            motor.kd = kd
            motor.has_target = True
            # A trajectory updates the target every cycle. Only the first update
            # leaves refresh/idle; resetting the counters here would never finish enabling.
            if motor.mode in ("refresh", "idle"):
                motor.mode = "configure"
                motor.configure_cycles = 0

    def prepare(self, send_ids: list[int]) -> None:
        """Start enabling before the first feedback sample.

        A move used to wait for a reply while the motors were only refreshed.
        Damiao motors go green, then red, when that stream never becomes an
        enable followed by a held position. Position commands stay at the
        measured angle until a target is set, so this does not drive toward zero.
        """

        with self._lock:
            for send_id in send_ids:
                motor = self.motors.get(send_id)
                if motor is not None and motor.mode in ("refresh", "idle"):
                    motor.mode = "configure"
                    motor.configure_cycles = 0

    def diagnose(self) -> str:
        with self._lock:
            error = self.last_error or "none"
            bus = self.last_bus or "none"
            others = ",".join(hex(can_id) for can_id in sorted(self.other_ids)) or "none"
            return (
                f"tx={self.tx_ok} tx_errors={self.tx_errors} rx={self.rx_ok} "
                f"rx_other={self.rx_other} other_ids={others} fd={self.fd} "
                f"bus_errors={self.bus_errors} last_error={error} last_bus={bus}"
            )

    def set_posforce_target(self, send_id: int, motor_position: float) -> None:
        with self._lock:
            motor = self.motors[send_id]
            motor.target = motor_position
            motor.has_target = True
            if motor.mode in ("refresh", "idle"):
                motor.mode = "configure"
                motor.configure_cycles = 0

    def hold(self, send_ids: list[int]) -> None:
        """Servo to the last measured position instead of the rest of the trajectory."""

        with self._lock:
            for send_id in send_ids:
                motor = self.motors.get(send_id)
                if motor is not None and motor.measured is not None:
                    motor.target = motor.measured

    def disable(self, send_ids: list[int]) -> None:
        packets: list[tuple[int, bytes]] = []
        with self._lock:
            for send_id in send_ids:
                motor = self.motors.get(send_id)
                if motor is None:
                    continue
                motor.mode = "idle"
                packets.append((send_id, command_packet(DISABLE)))
        self._send(packets)

    def _send(self, packets: list[tuple[int, bytes]]) -> None:
        if self._socket is None or not packets:
            return
        for arbitration_id, data in packets:
            self._transmit(arbitration_id, data)

    def _loop(self) -> None:
        while not self._stop.is_set():
            started = time.monotonic()
            self._exchange()
            remaining = CONTROL_PERIOD_S - (time.monotonic() - started)
            if remaining > 0:
                self._stop.wait(remaining)

    def _exchange(self) -> None:
        raw = self._socket
        if raw is None:
            return
        with self._lock:
            outbound = []
            for motor in self.motors.values():
                outbound.extend(commands_for(motor))
            recv_types = {motor.recv_id: motor.motor_type for motor in self.motors.values()}
        for arbitration_id, data in outbound:
            self._transmit(arbitration_id, data)
        deadline = time.monotonic() + 0.006
        while time.monotonic() < deadline:
            try:
                received = raw.recv(CANFD_MTU)
            except TimeoutError:
                break
            except OSError as exc:
                self._note_error(exc)
                return
            parsed = unpack_frame(received)
            if parsed is None:
                continue
            can_id, data, is_error = parsed
            if is_error:
                with self._lock:
                    self.bus_errors += 1
                    self.last_bus = bus_error_name(can_id)
                continue
            motor_type = recv_types.get(can_id)
            if motor_type is None:
                with self._lock:
                    self.rx_other += 1
                    if len(self.other_ids) < 16:
                        self.other_ids.add(can_id & 0x1FFFFFFF)
                continue
            state = parse_state(motor_type, data)
            if state is None:
                continue
            with self._lock:
                self.rx_ok += 1
                for motor in self.motors.values():
                    if motor.recv_id == can_id:
                        motor.measured = state.position
                        motor.velocity = state.velocity
                        motor.torque = state.torque
                        motor.error = state.error
                        break

    def _transmit(self, arbitration_id: int, data: bytes) -> None:
        raw = self._socket
        if raw is None:
            return
        try:
            raw.send(pack_canfd(arbitration_id, data) if self.fd else pack_classic(arbitration_id, data))
        except OSError as exc:
            self._note_error(exc)
            return
        with self._lock:
            self.tx_ok += 1

    def _note_error(self, exc: Exception) -> None:
        text = f"{type(exc).__name__}: {exc}"
        with self._lock:
            self.tx_errors += 1
            self.last_error = text[:180]
        LOGGER.warning("CAN send failed on %s: %s", self.interface, text)


_SESSIONS: dict[str, CanSession] = {}
_SESSIONS_LOCK = threading.Lock()


def acquire(interface: str) -> CanSession:
    with _SESSIONS_LOCK:
        session = _SESSIONS.get(interface)
        if session is None:
            session = CanSession(interface)
            session.start()
            _SESSIONS[interface] = session
        session.refs += 1
        return session


def release(session: CanSession) -> None:
    with _SESSIONS_LOCK:
        session.refs -= 1
        if session.refs <= 0:
            _SESSIONS.pop(session.interface, None)
            session.stop()
