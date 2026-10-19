"""One SocketCAN-FD session per interface, shared by the arm and gripper."""

from __future__ import annotations

import logging
import threading
import time
from dataclasses import dataclass, field

from openarm_viam.can.damiao import command_packet, mit_packet, parse_state, refresh_packet
from openarm_viam.can.damiao import DISABLE, ENABLE

LOGGER = logging.getLogger(__name__)
CONTROL_PERIOD_S = 0.01


@dataclass
class Motor:
    motor_type: str
    send_id: int
    recv_id: int
    mode: str = "refresh"
    enable_cycles: int = 0
    target: float = 0.0
    kp: float = 0.0
    kd: float = 0.0
    measured: float | None = None
    velocity: float = 0.0
    torque: float = 0.0
    error: int = 0


@dataclass
class CanSession:
    interface: str
    motors: dict[int, Motor] = field(default_factory=dict)
    refs: int = 0
    _lock: threading.Lock = field(default_factory=threading.Lock)
    _stop: threading.Event = field(default_factory=threading.Event)
    _thread: threading.Thread | None = None
    _bus: object | None = None

    def start(self) -> None:
        try:
            import can
        except ImportError as exc:
            raise RuntimeError("python-can is required to talk to OpenArm hardware") from exc
        self._bus = can.Bus(
            interface="socketcan",
            channel=self.interface,
            fd=True,
            bitrate=1_000_000,
            data_bitrate=5_000_000,
        )
        self._thread = threading.Thread(target=self._loop, name=f"openarm-{self.interface}", daemon=True)
        self._thread.start()
        LOGGER.info("opened CAN-FD session on %s", self.interface)

    def stop(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=1.0)
        bus = self._bus
        self._bus = None
        if bus is not None:
            bus.shutdown()  # type: ignore[attr-defined]
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

    def set_target(self, send_id: int, motor_position: float, kp: float, kd: float) -> None:
        with self._lock:
            motor = self.motors[send_id]
            motor.target = motor_position
            motor.kp = kp
            motor.kd = kd
            # A trajectory updates the target every cycle. Only the first update
            # leaves refresh/idle; resetting enable_cycles here would never finish enabling.
            if motor.mode in ("refresh", "idle"):
                motor.mode = "enable"
                motor.enable_cycles = 0

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
        bus = self._bus
        if bus is None or not packets:
            return
        import can

        for arbitration_id, data in packets:
            try:
                bus.send(  # type: ignore[attr-defined]
                    can.Message(
                        arbitration_id=arbitration_id,
                        data=data,
                        is_extended_id=False,
                        is_fd=True,
                        bitrate_switch=True,
                    )
                )
            except Exception:
                LOGGER.exception("failed to send CAN frame on %s", self.interface)
                return

    def _loop(self) -> None:
        while not self._stop.is_set():
            started = time.monotonic()
            self._exchange()
            remaining = CONTROL_PERIOD_S - (time.monotonic() - started)
            if remaining > 0:
                self._stop.wait(remaining)

    def _exchange(self) -> None:
        bus = self._bus
        if bus is None:
            return
        import can

        with self._lock:
            outbound = []
            for motor in self.motors.values():
                if motor.mode == "enable":
                    outbound.append((motor.send_id, command_packet(ENABLE)))
                    motor.enable_cycles += 1
                    if motor.enable_cycles >= 5:
                        motor.mode = "mit"
                elif motor.mode == "mit":
                    outbound.append(
                        (motor.send_id, mit_packet(motor.motor_type, motor.target, 0.0, motor.kp, motor.kd, 0.0))
                    )
                elif motor.mode == "disable":
                    outbound.append((motor.send_id, command_packet(DISABLE)))
                    motor.mode = "idle"
                elif motor.mode == "refresh":
                    arbitration_id, data = refresh_packet(motor.send_id)
                    outbound.append((arbitration_id, data))
            recv_types = {motor.recv_id: motor.motor_type for motor in self.motors.values()}
        for arbitration_id, data in outbound:
            try:
                bus.send(  # type: ignore[attr-defined]
                    can.Message(
                        arbitration_id=arbitration_id,
                        data=data,
                        is_extended_id=False,
                        is_fd=True,
                        bitrate_switch=True,
                    )
                )
            except Exception:
                LOGGER.exception("failed to send CAN frame on %s", self.interface)
                return
        deadline = time.monotonic() + 0.004
        while time.monotonic() < deadline:
            try:
                message = bus.recv(timeout=0.001)  # type: ignore[attr-defined]
            except Exception:
                LOGGER.exception("failed to read CAN frame on %s", self.interface)
                return
            if message is None:
                break
            motor_type = recv_types.get(message.arbitration_id)
            if motor_type is None:
                continue
            state = parse_state(motor_type, bytes(message.data))
            if state is None:
                continue
            with self._lock:
                for motor in self.motors.values():
                    if motor.recv_id == message.arbitration_id:
                        motor.measured = state.position
                        motor.velocity = state.velocity
                        motor.torque = state.torque
                        motor.error = state.error
                        break


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
