"""OpenArm 2.0 arm models.

Pose goals are intentionally unimplemented. The Viam motion service solves
inverse kinematics from the URDF returned by ``get_kinematics`` and calls
``move_to_joint_positions``.
"""

from __future__ import annotations

import asyncio
import math
from collections.abc import AsyncIterator
from typing import Any, ClassVar, Mapping, Sequence

from typing_extensions import Self
from viam.components.arm import Arm, JointPositions, Pose
from viam.operations import run_with_operation
from viam.proto.app.robot import ComponentConfig
from viam.proto.common import KinematicsFileFormat, ResourceName
from viam.resource.base import ResourceBase
from viam.resource.easy_resource import EasyResource
from viam.utils import ValueTypes, struct_to_dict

from openarm_viam import description as model
from openarm_viam.can.bus import CanSession, Motor, acquire, release
from openarm_viam.models.motion import (
    FEEDBACK_TIMEOUT_S,
    Motion,
    feedback_timeout,
    operation_from,
    wait_for_enable,
    wait_until,
)


class _ArmBase(Arm):
    def __init__(self, name: str):
        super().__init__(name)
        self.side = "right"
        self.positions = [0.0] * model.ARM_JOINTS
        self.motion = Motion()
        self.speed_scale = 1.0
        self.command_speeds = list(model.ARM_SPEEDS)
        self.kinematics = model.arm_urdf(self.side).encode("utf-8")

    def _apply_side(self, side: str) -> None:
        self.side = model.side_name(side)
        self.kinematics = model.arm_urdf(self.side).encode("utf-8")
        self.positions = model.clamp_arm(self.side, self.positions)

    def _apply_speed_scale(self, attributes: Mapping[str, Any]) -> None:
        scale = float(attributes.get("speed_scale", 1.0))
        if scale <= 0:
            raise ValueError("speed_scale must be positive")
        self.speed_scale = scale
        self.command_speeds = [speed * scale for speed in model.ARM_SPEEDS]

    async def get_end_position(
        self, *, extra: dict[str, Any] | None = None, timeout: float | None = None, **kwargs
    ) -> Pose:
        position, orientation = model.forward_kinematics(self.side, self.positions)
        return Pose(
            x=position[0],
            y=position[1],
            z=position[2],
            o_x=orientation[0],
            o_y=orientation[1],
            o_z=orientation[2],
            theta=orientation[3],
        )

    async def move_to_position(
        self, pose: Pose, *, extra: dict[str, Any] | None = None, timeout: float | None = None, **kwargs
    ):
        raise NotImplementedError(
            "OpenArm pose moves are handled by the Viam motion service, which calls move_to_joint_positions"
        )

    async def get_joint_positions(
        self, *, extra: dict[str, Any] | None = None, timeout: float | None = None, **kwargs
    ) -> JointPositions:
        # Viam joint positions are degrees. The model and the motors stay in radians.
        return JointPositions(values=[math.degrees(position) for position in self.positions])

    async def get_kinematics(self, *, extra: dict[str, Any] | None = None, timeout: float | None = None, **kwargs):
        return (KinematicsFileFormat.KINEMATICS_FILE_FORMAT_URDF, self.kinematics)

    async def is_moving(self) -> bool:
        return self.motion.moving

    async def get_status(self, *, timeout: float | None = None, **kwargs) -> Mapping[str, ValueTypes]:
        return {
            "side": self.side,
            "joints_degrees": [math.degrees(position) for position in self.positions],
            "moving": self.motion.moving,
        }

    @run_with_operation
    async def move_to_joint_positions(
        self,
        positions: JointPositions,
        *,
        extra: dict[str, Any] | None = None,
        timeout: float | None = None,
        **kwargs,
    ):
        goal = model.clamp_arm(self.side, _radians(positions.values))
        operation = operation_from(self, kwargs)

        async def update(values):
            self.positions = list(values)

        self.positions = await self.motion.run(self.positions, goal, self.command_speeds, update, operation)

    async def move_through_joint_positions(
        self,
        positions: list[JointPositions],
        options: Any = None,
        *,
        extra: dict[str, Any] | None = None,
        timeout: float | None = None,
        **kwargs,
    ):
        previous = self.command_speeds
        self.command_speeds = _speeds_for(options)
        try:
            for waypoint in positions:
                await self.move_to_joint_positions(waypoint, extra=extra, timeout=timeout, **kwargs)
        finally:
            self.command_speeds = previous

    async def move_through_joint_positions_streamed(
        self,
        batches: AsyncIterator[list[Arm.TrajectoryPoint]],
        *,
        extra: dict[str, Any] | None = None,
        timeout: float | None = None,
        **kwargs,
    ) -> AsyncIterator[Arm.TrajectoryUpdate]:
        sent = False
        async for batch in batches:
            for point in batch:
                await self.move_to_joint_positions(
                    JointPositions(values=point.positions), extra=extra, timeout=timeout, **kwargs
                )
            yield Arm.TrajectoryUpdate()
            sent = True
        if not sent:
            yield Arm.TrajectoryUpdate()

    async def get_3d_models(self, *, extra: dict[str, Any] | None = None, timeout: float | None = None, **kwargs):
        return {}

    async def set_manual_mode(
        self,
        manual_mode: bool,
        enabled_for: int = 0,
        *,
        extra: dict[str, Any] | None = None,
        timeout: float | None = None,
        **kwargs,
    ):
        if manual_mode:
            raise NotImplementedError("OpenArm has no software manual mode")

    async def get_manual_mode(
        self, *, extra: dict[str, Any] | None = None, timeout: float | None = None, **kwargs
    ) -> bool:
        return False

    async def get_properties(self, *, extra: dict[str, Any] | None = None, timeout: float | None = None, **kwargs):
        # Newer SDKs require this. The response type is not in older wheels.
        from viam.proto.component.arm import GetPropertiesResponse

        return GetPropertiesResponse(support_manual_mode=False, support_cartesian_commands=False)

    async def stop(self, *, extra: dict[str, Any] | None = None, timeout: float | None = None, **kwargs):
        self.motion.cancel()

    async def do_command(
        self, command: Mapping[str, ValueTypes], *, timeout: float | None = None, **kwargs
    ) -> Mapping[str, ValueTypes]:
        name = command.get("command")
        if name == "move_to_neutral":
            return await self._move_to_neutral(command, timeout=timeout, **kwargs)
        if name == "power_off":
            await self._power_off()
            return {"ok": True, "command": "power_off"}
        raise ValueError("unsupported DoCommand; use command='move_to_neutral' or command='power_off'")

    async def _move_to_neutral(
        self, command: Mapping[str, ValueTypes], *, timeout: float | None = None, **kwargs
    ) -> Mapping[str, ValueTypes]:
        scale = float(command.get("speed_scale", model.NEUTRAL_SPEED_SCALE))
        if scale <= 0:
            raise ValueError("speed_scale must be positive")
        previous = self.command_speeds
        self.command_speeds = [speed * scale for speed in previous]
        try:
            await self.move_to_joint_positions(
                JointPositions(values=[math.degrees(value) for value in model.NEUTRAL_POSITION_RAD]),
                timeout=timeout,
                **kwargs,
            )
        finally:
            self.command_speeds = previous
        return {
            "ok": True,
            "command": "move_to_neutral",
            "joints_degrees": [math.degrees(value) for value in self.positions],
            "speed_scale": scale,
        }

    async def _power_off(self) -> None:
        self.motion.cancel()


class FakeArm(_ArmBase, EasyResource):
    """Same kinematics and joint streaming as the hardware arm, with no CAN socket."""

    MODEL: ClassVar[str] = "viam-labs:openarm-2:fake-arm"  # type: ignore[assignment]

    @classmethod
    def new(cls, config: ComponentConfig, dependencies: Mapping[ResourceName, ResourceBase]) -> Self:
        arm = cls(config.name)
        arm.reconfigure(config, dependencies)
        return arm

    @classmethod
    def validate_config(cls, config: ComponentConfig) -> tuple[Sequence[str], Sequence[str]]:
        _require_side(struct_to_dict(config.attributes))
        return [], []

    def reconfigure(self, config: ComponentConfig, dependencies: Mapping[ResourceName, ResourceBase]):
        attributes = struct_to_dict(config.attributes)
        self._apply_side(_require_side(attributes))
        self._apply_speed_scale(attributes)


class OpenArm(_ArmBase, EasyResource):
    """7-DoF OpenArm 2.0 arm on a shared SocketCAN-FD bus."""

    MODEL: ClassVar[str] = "viam-labs:openarm-2:arm"  # type: ignore[assignment]

    def __init__(self, name: str):
        super().__init__(name)
        self.interface = ""
        self.gain_scale = 1.0
        self.hold_gain_scale = model.HOLD_GAIN_SCALE
        self.settle_gain = model.SETTLE_GAIN
        self.settle_passes = model.SETTLE_PASSES
        self.kp = list(model.ARM_KP)
        self.kd = list(model.ARM_KD)
        self.session: CanSession | None = None
        self.send_ids = list(model.SEND_IDS[: model.ARM_JOINTS])
        self._commanded: list[float] | None = None
        self._hold_bias: list[float] = [0.0] * model.ARM_JOINTS
        self.velocities = [0.0] * model.ARM_JOINTS
        self._settle_task: asyncio.Task[None] | None = None
        self._settle_token = 0

    @classmethod
    def new(cls, config: ComponentConfig, dependencies: Mapping[ResourceName, ResourceBase]) -> Self:
        arm = cls(config.name)
        arm.reconfigure(config, dependencies)
        return arm

    @classmethod
    def validate_config(cls, config: ComponentConfig) -> tuple[Sequence[str], Sequence[str]]:
        _hardware_attributes(struct_to_dict(config.attributes))
        return [], []

    def reconfigure(self, config: ComponentConfig, dependencies: Mapping[ResourceName, ResourceBase]):
        self.close_hardware()
        attributes = _hardware_attributes(struct_to_dict(config.attributes))
        self._apply_side(attributes["side"])
        self._apply_speed_scale(attributes)
        self.interface = attributes["can_interface"]
        self.gain_scale = float(attributes.get("gain_scale", 1.0))
        self.hold_gain_scale = float(attributes.get("hold_gain_scale", model.HOLD_GAIN_SCALE))
        self.settle_gain = float(attributes.get("settle_gain", model.SETTLE_GAIN))
        if self.settle_gain < 0:
            raise ValueError("settle_gain must be non-negative")
        if self.settle_gain > model.SETTLE_GAIN_MAX:
            self.logger.warning(
                "settle_gain %.2f is capped at %.2f — higher values make the arm rock",
                self.settle_gain,
                model.SETTLE_GAIN_MAX,
            )
            self.settle_gain = model.SETTLE_GAIN_MAX
        self.settle_passes = int(attributes.get("settle_passes", model.SETTLE_PASSES))
        if self.settle_passes < 0:
            raise ValueError("settle_passes must be non-negative")
        self.kp = _gains(attributes, "kp", model.ARM_KP)
        self.kd = _gains(attributes, "kd", model.ARM_KD)
        self.session = acquire(self.interface)
        for index in range(model.ARM_JOINTS):
            self.session.add_motor(
                Motor(
                    motor_type=model.MOTOR_TYPES[index],
                    send_id=model.SEND_IDS[index],
                    recv_id=model.RECV_IDS[index],
                )
            )
        self.logger.info("OpenArm %s using %s side %s", self.name, self.interface, self.side)

    async def get_end_position(
        self, *, extra: dict[str, Any] | None = None, timeout: float | None = None, **kwargs
    ) -> Pose:
        self._read_feedback()
        return await super().get_end_position(extra=extra, timeout=timeout, **kwargs)

    async def get_joint_positions(
        self, *, extra: dict[str, Any] | None = None, timeout: float | None = None, **kwargs
    ) -> JointPositions:
        self._read_feedback()
        return await super().get_joint_positions(extra=extra, timeout=timeout, **kwargs)

    @run_with_operation
    async def move_to_joint_positions(
        self,
        positions: JointPositions,
        *,
        extra: dict[str, Any] | None = None,
        timeout: float | None = None,
        **kwargs,
    ):
        self._cancel_settle()
        session = self._session()
        # Do not re-run clear/enable when the arm is already holding. That path
        # was dropping base-joint torque between single-joint jogs.
        if not session.commanding(self.send_ids):
            session.prepare(self.send_ids)
            await wait_until(
                lambda: self._feedback_ready(),
                FEEDBACK_TIMEOUT_S,
                feedback_timeout(self.interface, session),
            )
            await wait_for_enable(
                self.interface,
                session,
                [(f"joint{index + 1}", send_id) for index, send_id in enumerate(self.send_ids)],
            )
            # Freshly enabled motors hold their measured angle; the old command is stale.
            self._commanded = None
        self._read_feedback()
        measured = list(self.positions)
        # Start from the last command, not the encoders. A PD-held joint sits below
        # its target under gravity; re-targeting to the measured angle removes the
        # holding torque and the joint falls until it finds a new, lower equilibrium.
        start = list(self._commanded) if self._commanded is not None else measured
        requested = model.clamp_arm(self.side, _radians(positions.values))
        holding = self._holding_for_move(start, measured, requested)
        goal = [start[index] if holding[index] else requested[index] for index in range(len(requested))]
        operation = operation_from(self, kwargs)

        async def update(values):
            if not self.motion.moving:
                return
            self.positions = list(values)
            self._command(values, holding)

        final = await self.motion.run(start, goal, self.command_speeds, update, operation)
        self.positions = list(final)
        # A cancelled move already left its last command on the motors. Commanding
        # the full goal here would jump past a stop.
        if any(abs(reached - desired) > 1e-9 for reached, desired in zip(final, goal)):
            return
        self._finish_move(goal, holding)

    async def stop(self, *, extra: dict[str, Any] | None = None, timeout: float | None = None, **kwargs):
        # The motors keep their last command, which freezes the arm where the
        # trajectory was. Re-targeting to the measured pose would drop held joints.
        self._cancel_settle()
        await super().stop(extra=extra, timeout=timeout, **kwargs)

    async def _power_off(self) -> None:
        self._cancel_settle()
        self.motion.cancel()
        session = self.session
        if session is None:
            return
        self._read_feedback()
        self._commanded = None
        self._hold_bias = [0.0] * model.ARM_JOINTS
        session.disable(self.send_ids)

    def _cancel_settle(self) -> None:
        self._settle_token += 1
        task = self._settle_task
        self._settle_task = None
        if task is not None and not task.done():
            task.cancel()

    def _holding_for_move(
        self, reference: Sequence[float], measured: Sequence[float], requested: Sequence[float]
    ) -> list[bool]:
        return [
            abs(want - last) <= model.HOLD_EPSILON_RAD or abs(want - now) <= model.HOLD_ECHO_RAD
            for last, now, want in zip(reference, measured, requested)
        ]

    def _finish_move(self, goal: Sequence[float], holding: Sequence[bool]) -> None:
        """Hold the trajectory's final command; optionally start background settle."""

        self._cancel_settle()
        self._hold_bias = [0.0] * len(goal)
        self._command(goal, holding)
        if self.settle_gain <= 0 or self.settle_passes <= 0 or not any(holding):
            return
        self._read_feedback()
        if not model.holding_needs_settle(goal, self.positions, holding):
            return
        token = self._settle_token
        self._settle_task = asyncio.create_task(
            self._settle_to_goal(list(goal), list(holding), token),
            name=f"openarm-settle-{self.name}",
        )

    async def _settle_to_goal(self, goal: Sequence[float], holding: Sequence[bool], token: int) -> None:
        """Still-gated bias trim for held joints that already drooped.

        The moved joints are never biased. A new move, stop, or power_off cancels
        this task so consecutive jogs are not serialized behind settle.
        """

        try:
            for _ in range(self.settle_passes):
                if token != self._settle_token:
                    return
                if not await self._wait_nearly_still(token):
                    return
                if token != self._settle_token:
                    return
                self._read_feedback()
                if model.holding_on_target(goal, self.positions, holding):
                    return
                self._hold_bias = model.accumulate_hold_bias(
                    self._hold_bias, goal, self.positions, self.settle_gain, holding
                )
                if token != self._settle_token:
                    return
                self._command([desired + bias for desired, bias in zip(goal, self._hold_bias)], holding)
            if token == self._settle_token:
                self._read_feedback()
        except asyncio.CancelledError:
            raise
        except Exception:
            self.logger.exception("background settle failed on %s", self.name)

    async def _wait_nearly_still(self, token: int) -> bool:
        await asyncio.sleep(model.SETTLE_PAUSE_S)
        if token != self._settle_token:
            return False
        deadline = asyncio.get_running_loop().time() + model.SETTLE_STILL_TIMEOUT_S
        while asyncio.get_running_loop().time() < deadline:
            if token != self._settle_token:
                return False
            self._read_feedback()
            if max(abs(velocity) for velocity in self.velocities) <= model.SETTLE_STILL_VEL_RAD_S:
                return True
            await asyncio.sleep(0.02)
        return token == self._settle_token

    def close_hardware(self) -> None:
        self._cancel_settle()
        self._commanded = None
        self._hold_bias = [0.0] * model.ARM_JOINTS
        session = self.session
        self.session = None
        if session is None:
            return
        session.disable(self.send_ids)
        session.drop_motors(self.send_ids)
        release(session)

    async def close(self):
        self._cancel_settle()
        self.motion.cancel()
        self.close_hardware()

    def _session(self) -> CanSession:
        if self.session is None:
            raise RuntimeError("arm is not connected to a CAN interface")
        return self.session

    def _feedback_ready(self) -> bool:
        session = self.session
        if session is None:
            return False
        return all(session.measured(send_id) is not None for send_id in self.send_ids)

    def _read_feedback(self) -> None:
        session = self.session
        if session is None or not self._feedback_ready():
            return
        motor_positions = [session.measured(send_id) or 0.0 for send_id in self.send_ids]
        self.velocities = [session.velocity(send_id) for send_id in self.send_ids]
        self.positions = model.clamp_arm(self.side, model.kinematic_from_motor(self.side, motor_positions))

    def _command(self, kinematic: Sequence[float], holding: Sequence[bool] | None = None) -> None:
        session = self._session()
        self._commanded = list(kinematic)
        motor_positions = model.motor_from_kinematic(self.side, kinematic)
        for index, send_id in enumerate(self.send_ids):
            scale = self.gain_scale
            if holding is not None and holding[index]:
                scale *= self.hold_gain_scale
            session.set_target(
                send_id,
                motor_positions[index],
                self.kp[index] * scale,
                self.kd[index] * scale,
            )


def _radians(values: Sequence[float]) -> list[float]:
    return [math.radians(value) for value in values]


def _speeds_for(options: Any) -> list[float]:
    speeds = list(model.ARM_SPEEDS)
    if options is None:
        return speeds
    per_joint = list(getattr(options, "max_vel_degs_per_sec_joints", []) or [])
    if per_joint:
        capped = []
        for index, speed in enumerate(speeds):
            cap = math.radians(per_joint[index]) if index < len(per_joint) else speed
            capped.append(min(speed, cap) if cap > 0 else speed)
        return capped
    has_field = getattr(options, "HasField", None)
    if callable(has_field) and has_field("max_vel_degs_per_sec"):
        cap = math.radians(options.max_vel_degs_per_sec)
        if cap > 0:
            return [min(speed, cap) for speed in speeds]
    return speeds


def _require_side(attributes: Mapping[str, Any]) -> str:
    side = attributes.get("side")
    if side not in ("left", "right"):
        raise ValueError("side must be 'left' or 'right'")
    return str(side)


def _hardware_attributes(attributes: Mapping[str, Any]) -> dict[str, Any]:
    side = _require_side(attributes)
    interface = attributes.get("can_interface")
    if not interface or not isinstance(interface, str):
        raise ValueError("can_interface is required, for example 'can0'")
    return {"side": side, "can_interface": interface, **dict(attributes)}


def _gains(attributes: Mapping[str, Any], key: str, default: Sequence[float]) -> list[float]:
    value = attributes.get(key, default)
    gains = [float(item) for item in value]
    if len(gains) != model.ARM_JOINTS:
        raise ValueError(f"{key} must contain {model.ARM_JOINTS} values")
    return gains
