"""OpenArm 2.0 kinematic data used by the arm and gripper models.

Numbers are taken from the public OpenArm v2.0 description (joint origins,
axes, limits, and body mount points) and from the OpenArm driver MIT gains.
Link collision uses boxes sized from those link lengths. Viam's URDF importer
rejects standalone cylinders, and a model that fails to parse is omitted from
the frame system. The module does not vendor mesh files.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Iterable, Sequence
ARM_JOINTS = 7

# Canonical (right-arm) joint data from openarm_description v2.0.
_ORIGINS = (
    (0.0, -0.0625, 0.0),
    (0.0, -0.0600000000000001, 0.0),
    (0.0, 0.0, -0.06625000000000079),
    (0.0, 0.0, -0.15375),
    (0.0, 0.0, -0.0955000000000005),
    (0.0, 0.0, -0.120499999999998),
    (0.0, 0.0, 0.0),
)
_AXES = (
    (0.0, -1.0, 0.0),
    (-1.0, 0.0, 0.0),
    (0.0, 0.0, -1.0),
    (0.0, -1.0, 0.0),
    (0.0, 0.0, -1.0),
    (0.0, 1.0, 0.0),
    (1.0, 0.0, 0.0),
)
_LIMITS = (
    (-1.3963, 3.4907),
    (-0.17453, 3.3161),
    (-1.5708, 1.5708),
    (0.0, 2.4435),
    (-1.5708, 1.5708),
    (-0.7854, 0.7854),
    (-1.5708, 1.5708),
)
_EFFORT = (40.0, 40.0, 27.0, 27.0, 7.0, 7.0, 7.0)

# Soft default streaming speeds (rad/s). Multiply with the speed_scale attribute.
ARM_SPEEDS = (0.55, 0.55, 0.8, 0.65, 1.0, 1.0, 1.0)
# Hardware rate caps written into the URDF for motion planning.
ARM_MAX_SPEEDS = (1.8, 1.8, 3.3, 2.3, 3.5, 3.5, 3.5)
# URDF home used by the move_to_neutral DoCommand (joint-space, not motion-planned).
NEUTRAL_POSITION_RAD = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
# Extra multiplier on the configured joint speeds for a careful move_to_neutral.
NEUTRAL_SPEED_SCALE = 0.35

# MIT gains. Stiffer than the ROS defaults for gravity hold, softer and more
# damped than the raw OpenArm driver values to avoid pose-dependent buzzing.
ARM_KP = (140.0, 140.0, 120.0, 120.0, 22.0, 22.0, 22.0)
ARM_KD = (3.2, 3.2, 2.8, 2.8, 1.6, 1.6, 1.6)
# Extra stiffness for joints that are not part of the current move.
HOLD_GAIN_SCALE = 1.15
# Joints whose goal is within this many radians of the start are treated as holds.
HOLD_EPSILON_RAD = 1e-3
# After the trajectory, close the loop on measured pose while the arm is still:
# bias += settle_gain * (goal - measured), command goal + bias, repeat until
# within SETTLE_EPS or max passes, then freeze. Updates only when nearly still
# so this is not the live high-rate outer loop that rocked the arm.
SETTLE_GAIN = 0.55
SETTLE_GAIN_MAX = 1.0
SETTLE_PASSES = 8
SETTLE_EPS_RAD = math.radians(0.2)
SETTLE_PAUSE_S = 0.12
SETTLE_STILL_VEL_RAD_S = 0.025
SETTLE_STILL_TIMEOUT_S = 0.6
SETTLE_MAX_BIAS_RAD = math.radians(8.0)
# Position-force limits from the OpenArm driver. Torque is a fraction of the 4.5 Nm motor rating.
GRIPPER_POS_SPEED = 50.0
GRIPPER_POS_TORQUE = 1.0 / 4.5

# After openarm-can-zero-position-calibration, Damiao firmware zero matches the
# URDF zero (same convention as openarm_ros2). Arm joints therefore need no
# software offset. Earlier non-zero constants (especially joint4 ≈ -100°) made
# a straight arm look bent in the Viam scene. Index 7 is the gripper motor;
# open amount still uses GRIPPER_OPEN_TRAVEL below, not this slot.
RIGHT_OFFSETS = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
LEFT_OFFSETS = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)

MOTOR_TYPES = ("DM8009", "DM8009", "DM4340", "DM4340", "DM4310", "DM4310", "DM4310", "DM3507")
SEND_IDS = (0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08)
RECV_IDS = (0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17, 0x18)

# Joints whose frames the v2 left-arm preset reflects across Y.
_LEFT_REFLECT_JOINTS = {0, 1, 2, 4, 5, 6}

# Body mount points, meters, from the v2 body reference points.
RIGHT_MOUNT_M = (0.0, -0.030, 0.697)
LEFT_MOUNT_M = (0.0, 0.030, 0.697)
# Commented example in the v2 body reference points. Calibrate on hardware.
HEAD_STEREO_M = (0.12, 0.0, 0.82)

# End-effector mount on link7, then the pinch grasp frame, both in meters.
EE_MOUNT_M = (0.0205, 0.0, 0.0)
GRASP_M = (-0.02193, 0.0, -0.138)

# Pinch joint: 0 is closed, 1 is open. The real finger swing is about 0.785 rad;
# commanding a full radian over-approximates the finger sweep for planning.
GRIPPER_CLOSED = 0.0
GRIPPER_OPEN = 1.0
GRIPPER_SPEED = 3.5
# Motor travel from closed to open. Sign differs by side in the driver limits.
GRIPPER_OPEN_TRAVEL = 1.047198
RIGHT_GRIPPER_SIGN = -1.0
LEFT_GRIPPER_SIGN = 1.0

_UPPER_RADIUS = 0.05
_WRIST_RADIUS = 0.035


@dataclass(frozen=True)
class JointSpec:
    origin: tuple[float, float, float]
    axis: tuple[float, float, float]
    lower: float
    upper: float
    effort: float
    velocity: float


def side_name(side: str) -> str:
    if side not in ("left", "right"):
        raise ValueError("side must be 'left' or 'right'")
    return side


def offsets_for(side: str) -> tuple[float, ...]:
    return LEFT_OFFSETS if side_name(side) == "left" else RIGHT_OFFSETS


def gripper_sign(side: str) -> float:
    return LEFT_GRIPPER_SIGN if side_name(side) == "left" else RIGHT_GRIPPER_SIGN


def mount_mm(side: str) -> tuple[float, float, float]:
    mount = LEFT_MOUNT_M if side_name(side) == "left" else RIGHT_MOUNT_M
    return tuple(value * 1000.0 for value in mount)  # type: ignore[return-value]


def head_stereo_mm() -> tuple[float, float, float]:
    return tuple(value * 1000.0 for value in HEAD_STEREO_M)  # type: ignore[return-value]


def gripper_tcp_mm() -> tuple[float, float, float]:
    return (
        (EE_MOUNT_M[0] + GRASP_M[0]) * 1000.0,
        (EE_MOUNT_M[1] + GRASP_M[1]) * 1000.0,
        (EE_MOUNT_M[2] + GRASP_M[2]) * 1000.0,
    )


def arm_joints(side: str) -> tuple[JointSpec, ...]:
    specs = []
    for index, origin in enumerate(_ORIGINS):
        axis = _AXES[index]
        lower, upper = _LIMITS[index]
        if side_name(side) == "left" and index in _LEFT_REFLECT_JOINTS:
            # v2 reflects these joints across Y. A reflected joint negates its
            # angle even when the axis has no Y component, which is joint2.
            origin = (origin[0], -origin[1], origin[2])
            axis = (axis[0], -axis[1], axis[2])
            lower, upper = -upper, -lower
        specs.append(
            JointSpec(
                origin=origin,
                axis=axis,
                lower=lower,
                upper=upper,
                effort=_EFFORT[index],
                velocity=ARM_MAX_SPEEDS[index],
            )
        )
    return tuple(specs)


def clamp_arm(side: str, values: Sequence[float]) -> list[float]:
    specs = arm_joints(side)
    if len(values) != len(specs):
        raise ValueError(f"arm expects {len(specs)} joint positions, got {len(values)}")
    return [min(spec.upper, max(spec.lower, value)) for spec, value in zip(specs, values)]


def holding_joints(
    start: Sequence[float], goal: Sequence[float], epsilon: float = HOLD_EPSILON_RAD
) -> list[bool]:
    """True for each joint whose goal matches the start within ``epsilon`` radians."""

    if len(start) != len(goal):
        raise ValueError("start and goal must have the same length")
    return [abs(current - desired) <= epsilon for current, desired in zip(start, goal)]


def settle_targets(
    goal: Sequence[float],
    measured: Sequence[float],
    settle_gain: float,
    holding: Sequence[bool] | None = None,
) -> list[float]:
    """Command ``goal + settle_gain * (goal - measured)`` (capped)."""

    bias = accumulate_hold_bias([0.0] * len(goal), goal, measured, settle_gain)
    _ = holding
    return [desired + offset for desired, offset in zip(goal, bias)]


def accumulate_hold_bias(
    bias: Sequence[float],
    goal: Sequence[float],
    measured: Sequence[float],
    settle_gain: float,
) -> list[float]:
    """Add ``settle_gain * (goal - measured)`` onto an existing frozen bias."""

    if len(goal) != len(measured) or len(bias) != len(goal):
        raise ValueError("bias, goal, and measured must have the same length")
    updated = []
    for offset, desired, current in zip(bias, goal, measured):
        next_bias = offset + settle_gain * (desired - current)
        next_bias = max(-SETTLE_MAX_BIAS_RAD, min(SETTLE_MAX_BIAS_RAD, next_bias))
        updated.append(next_bias)
    return updated


def motor_from_kinematic(side: str, kinematic: Sequence[float]) -> list[float]:
    offsets = offsets_for(side)
    clamped = clamp_arm(side, kinematic)
    return [angle + offsets[index] for index, angle in enumerate(clamped)]


def kinematic_from_motor(side: str, motor: Sequence[float]) -> list[float]:
    offsets = offsets_for(side)
    if len(motor) < ARM_JOINTS:
        raise ValueError(f"expected at least {ARM_JOINTS} motor positions")
    return [motor[index] - offsets[index] for index in range(ARM_JOINTS)]


def gripper_motor(side: str, open_amount: float) -> float:
    amount = min(GRIPPER_OPEN, max(GRIPPER_CLOSED, open_amount))
    return offsets_for(side)[ARM_JOINTS] + gripper_sign(side) * amount * GRIPPER_OPEN_TRAVEL


def gripper_open_amount(side: str, motor: float) -> float:
    signed = gripper_sign(side) * GRIPPER_OPEN_TRAVEL
    amount = (motor - offsets_for(side)[ARM_JOINTS]) / signed
    return min(GRIPPER_OPEN, max(GRIPPER_CLOSED, amount))


def arm_urdf(side: str) -> str:
    side = side_name(side)
    prefix = f"openarm_{side}"
    specs = arm_joints(side)
    lines = [
        '<?xml version="1.0"?>',
        f'<robot name="{prefix}">',
    ]
    for index in range(ARM_JOINTS + 1):
        name = f"{prefix}_base_link" if index == 0 else f"{prefix}_link{index}"
        lines.append(f'  <link name="{name}">')
        # Boxes, not cylinders. Viam's URDF importer rejects a standalone cylinder,
        # and a model that fails to parse is left out of the frame system.
        if index < ARM_JOINTS:
            thickness = _UPPER_RADIUS * 2 if index < 4 else _WRIST_RADIUS * 2
            lines.append(_box_collision(specs[index].origin, thickness))
        else:
            lines.append(_box_collision((0.0, 0.0, 0.0), 0.04))
        lines.append("  </link>")
    for index, spec in enumerate(specs):
        parent = f"{prefix}_base_link" if index == 0 else f"{prefix}_link{index}"
        child = f"{prefix}_link{index + 1}"
        ox, oy, oz = spec.origin
        ax, ay, az = spec.axis
        lines.append(f'  <joint name="{prefix}_joint{index + 1}" type="revolute">')
        lines.append(f'    <parent link="{parent}"/>')
        lines.append(f'    <child link="{child}"/>')
        lines.append(f'    <origin xyz="{_fmt(ox)} {_fmt(oy)} {_fmt(oz)}" rpy="0 0 0"/>')
        lines.append(f'    <axis xyz="{_fmt(ax)} {_fmt(ay)} {_fmt(az)}"/>')
        lines.append(
            f'    <limit lower="{_fmt(spec.lower)}" upper="{_fmt(spec.upper)}" '
            f'effort="{_fmt(spec.effort)}" velocity="{_fmt(spec.velocity)}"/>'
        )
        lines.append("  </joint>")
    lines.append("</robot>")
    return "\n".join(lines) + "\n"


def gripper_urdf(side: str) -> str:
    """Static gripper volume. The pinch itself is not a frame-system joint.

    Viam needs exactly one leaf frame. The two finger joints in the OpenArm
    description are a branch, so this model keeps a single box at the grasp
    frame configured on the gripper component.
    """

    side = side_name(side)
    prefix = f"openarm_{side}_gripper"
    lines = [
        '<?xml version="1.0"?>',
        f'<robot name="{prefix}">',
        f'  <link name="{prefix}_base_link">',
        '    <collision><origin xyz="0 0 0" rpy="0 0 0"/>',
        '      <geometry><box size="0.08 0.05 0.04"/></geometry></collision>',
        "  </link>",
        "</robot>",
    ]
    return "\n".join(lines) + "\n"


def forward_kinematics(side: str, joints: Sequence[float]) -> tuple[tuple[float, float, float], tuple[float, float, float, float]]:
    """Return the link7 origin in millimeters and an orientation vector in degrees."""

    specs = arm_joints(side)
    angles = clamp_arm(side, joints)
    transform = _identity()
    for spec, angle in zip(specs, angles):
        transform = _matmul(transform, _translation(*spec.origin))
        transform = _matmul(transform, _rotation_about(spec.axis, angle))
    position_m = (transform[0][3], transform[1][3], transform[2][3])
    position_mm = tuple(value * 1000.0 for value in position_m)
    return position_mm, _orientation_vector(transform)  # type: ignore[return-value]


def _box_collision(origin: tuple[float, float, float], thickness: float) -> str:
    """Axis-aligned box covering the link segment. Joint origins are axis-aligned."""

    size = [thickness, thickness, thickness]
    for index, value in enumerate(origin):
        if abs(value) > 1e-9:
            size[index] = max(abs(value), thickness)
    center = tuple(value / 2.0 for value in origin)
    sx, sy, sz = size
    cx, cy, cz = center
    return (
        f'    <collision><origin xyz="{_fmt(cx)} {_fmt(cy)} {_fmt(cz)}" rpy="0 0 0"/>'
        f'<geometry><box size="{_fmt(sx)} {_fmt(sy)} {_fmt(sz)}"/></geometry></collision>'
    )


def _fmt(value: float) -> str:
    return f"{value:.8g}"


def _dot(left: Iterable[float], right: Iterable[float]) -> float:
    return sum(a * b for a, b in zip(left, right))


def _identity() -> list[list[float]]:
    return [
        [1.0, 0.0, 0.0, 0.0],
        [0.0, 1.0, 0.0, 0.0],
        [0.0, 0.0, 1.0, 0.0],
        [0.0, 0.0, 0.0, 1.0],
    ]


def _translation(x: float, y: float, z: float) -> list[list[float]]:
    transform = _identity()
    transform[0][3] = x
    transform[1][3] = y
    transform[2][3] = z
    return transform


def _rotation_about(axis: tuple[float, float, float], angle: float) -> list[list[float]]:
    length = math.sqrt(_dot(axis, axis))
    x, y, z = (component / length for component in axis)
    c = math.cos(angle)
    s = math.sin(angle)
    one_c = 1.0 - c
    return [
        [c + x * x * one_c, x * y * one_c - z * s, x * z * one_c + y * s, 0.0],
        [y * x * one_c + z * s, c + y * y * one_c, y * z * one_c - x * s, 0.0],
        [z * x * one_c - y * s, z * y * one_c + x * s, c + z * z * one_c, 0.0],
        [0.0, 0.0, 0.0, 1.0],
    ]


def _matmul(left: Sequence[Sequence[float]], right: Sequence[Sequence[float]]) -> list[list[float]]:
    result = [[0.0, 0.0, 0.0, 0.0] for _ in range(4)]
    for row in range(4):
        for col in range(4):
            result[row][col] = sum(left[row][k] * right[k][col] for k in range(4))
    return result


def _orientation_vector(transform: Sequence[Sequence[float]]) -> tuple[float, float, float, float]:
    rotation = tuple(tuple(transform[row][col] for col in range(3)) for row in range(3))
    trace = rotation[0][0] + rotation[1][1] + rotation[2][2]
    angle = math.acos(max(-1.0, min(1.0, (trace - 1.0) / 2.0)))
    if angle < 1e-8:
        return (0.0, 0.0, 1.0, 0.0)
    axis = (
        rotation[2][1] - rotation[1][2],
        rotation[0][2] - rotation[2][0],
        rotation[1][0] - rotation[0][1],
    )
    axis = _normalize(axis)
    return (axis[0], axis[1], axis[2], math.degrees(angle))


def _normalize(vector: tuple[float, float, float]) -> tuple[float, float, float]:
    length = math.sqrt(_dot(vector, vector))
    if length < 1e-12:
        return (0.0, 0.0, 1.0)
    return (vector[0] / length, vector[1] / length, vector[2] / length)

