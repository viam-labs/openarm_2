"""Damiao motor MIT frames, matching the OpenArm CAN library packing."""

from __future__ import annotations

from dataclasses import dataclass

CLEAR_ERROR = 0xFB
ENABLE = 0xFC
DISABLE = 0xFD
SET_ZERO = 0xFE

# Upper nibble of byte 0 in every state reply.
STATUS_DISABLED = 0x0
STATUS_ENABLED = 0x1
STATUS_NAMES = {
    0x0: "disabled",
    0x1: "enabled",
    0x8: "overvoltage",
    0x9: "undervoltage",
    0xA: "overcurrent",
    0xB: "MOS overheat",
    0xC: "coil overheat",
    0xD: "communication lost",
    0xE: "overload",
}

# Damiao register 10. MIT is the arm mode; the gripper uses position-force.
CTRL_MODE = 10
MODE_MIT = 1
MODE_POS_FORCE = 4

# Position, velocity, and torque spans used by the MIT float encoding.
LIMITS = {
    "DM3507": (12.5, 50.0, 5.0),
    "DM8009": (12.5, 45.0, 54.0),
    "DM4340": (12.5, 10.0, 28.0),
    "DM4310": (12.5, 30.0, 10.0),
}


@dataclass(frozen=True)
class MotorState:
    position: float
    velocity: float
    torque: float
    temperature_mos: int
    temperature_rotor: int
    error: int


def mit_packet(motor_type: str, position: float, velocity: float, kp: float, kd: float, torque: float) -> bytes:
    p_max, v_max, t_max = LIMITS[motor_type]
    position_u = _to_uint(position, -p_max, p_max, 16)
    velocity_u = _to_uint(velocity, -v_max, v_max, 12)
    kp_u = _to_uint(kp, 0.0, 500.0, 12)
    kd_u = _to_uint(kd, 0.0, 5.0, 12)
    torque_u = _to_uint(torque, -t_max, t_max, 12)
    return bytes(
        (
            (position_u >> 8) & 0xFF,
            position_u & 0xFF,
            velocity_u >> 4,
            ((velocity_u & 0xF) << 4) | ((kp_u >> 8) & 0xF),
            kp_u & 0xFF,
            kd_u >> 4,
            ((kd_u & 0xF) << 4) | ((torque_u >> 8) & 0xF),
            torque_u & 0xFF,
        )
    )


def is_param_reply(data: bytes) -> bool:
    """Register read/write replies share the motor's reply ID with state frames.

    A state frame with these bytes would put the motor at about -12.47 rad,
    outside every OpenArm joint, so the pattern is unambiguous in practice.
    """

    return len(data) >= 4 and data[1] == 0x00 and data[2] in (0x33, 0x55)


def status_name(code: int) -> str:
    return STATUS_NAMES.get(code, f"status 0x{code:x}")


def parse_state(motor_type: str, data: bytes) -> MotorState | None:
    if len(data) < 8 or is_param_reply(data):
        return None
    p_max, v_max, t_max = LIMITS[motor_type]
    position_u = (data[1] << 8) | data[2]
    velocity_u = (data[3] << 4) | (data[4] >> 4)
    torque_u = ((data[4] & 0xF) << 8) | data[5]
    return MotorState(
        position=_from_uint(position_u, -p_max, p_max, 16),
        velocity=_from_uint(velocity_u, -v_max, v_max, 12),
        torque=_from_uint(torque_u, -t_max, t_max, 12),
        temperature_mos=data[6],
        temperature_rotor=data[7],
        error=data[0] >> 4,
    )


def command_packet(command: int) -> bytes:
    return bytes((0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, command))


def refresh_packet(send_id: int) -> tuple[int, bytes]:
    return 0x7FF, bytes((send_id & 0xFF, (send_id >> 8) & 0xFF, 0xCC, 0x00, 0x00, 0x00, 0x00, 0x00))


def control_mode_packet(send_id: int, mode: int) -> tuple[int, bytes]:
    """Write the control-mode register. The motor ignores MIT frames in any other mode."""

    return 0x7FF, bytes((send_id & 0xFF, (send_id >> 8) & 0xFF, 0x55, CTRL_MODE, mode & 0xFF, 0x00, 0x00, 0x00))


def posforce_packet(position: float, speed_rad_s: float, torque_pu: float) -> bytes:
    """Position-force frame: little-endian float position, then speed and torque limits."""

    import struct

    speed = min(100.0, max(0.0, speed_rad_s))
    torque = min(1.0, max(0.0, torque_pu))
    return struct.pack("<fHH", float(position), int(speed * 100), int(torque * 10000))


def _to_uint(value: float, lower: float, upper: float, bits: int) -> int:
    value = min(upper, max(lower, value))
    span = upper - lower
    normalized = (value - lower) / span
    return int(normalized * ((1 << bits) - 1))


def _from_uint(value: int, lower: float, upper: float, bits: int) -> float:
    span = upper - lower
    normalized = value / ((1 << bits) - 1)
    return normalized * span + lower
