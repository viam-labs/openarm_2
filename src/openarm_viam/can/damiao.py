"""Damiao motor MIT frames, matching the OpenArm CAN library packing."""

from __future__ import annotations

from dataclasses import dataclass

ENABLE = 0xFC
DISABLE = 0xFD
SET_ZERO = 0xFE

# Position, velocity, and torque spans used by the MIT float encoding.
LIMITS = {
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


def parse_state(motor_type: str, data: bytes) -> MotorState | None:
    if len(data) < 8:
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


def _to_uint(value: float, lower: float, upper: float, bits: int) -> int:
    value = min(upper, max(lower, value))
    span = upper - lower
    normalized = (value - lower) / span
    return int(normalized * ((1 << bits) - 1))


def _from_uint(value: int, lower: float, upper: float, bits: int) -> float:
    span = upper - lower
    normalized = value / ((1 << bits) - 1)
    return normalized * span + lower
