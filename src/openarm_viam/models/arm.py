"""OpenArm 2.0 arm models.

Pose goals are intentionally unimplemented. The Viam motion service solves
inverse kinematics from the URDF returned by ``get_kinematics`` and calls
``move_to_joint_positions``.
"""

from __future__ import annotations

import math
from collections.abc import AsyncIterator
from typing import Any, ClassVar, Mapping, Sequence

from typing_extensions import Self
from viam.components.arm import Arm, JointPositions, Pose
from viam.operations import run_with_operation
from viam.proto.app.robot import ComponentConfig
from viam.proto.common import KinematicsFileFormat
from viam.proto.common import ResourceName
from viam.resource.base import ResourceBase
from viam.resource.easy_resource import EasyResource
from viam.utils import ValueTypes, struct_to_dict

from openarm_viam import description as model
from openarm_viam.can.bus import CanSession, Motor, acquire, release
from openarm_viam.models.motion import Motion, feedback_timeout, operation_from, wait_for_enable, wait_until


class _ArmBase(Arm):
    def __init__(self, name: str):
        super().__init__(name)
        self.side = "right"
        self.positions = [0.0] * model.ARM_JOINTS
        self.motion = Motion()
        self.command_speeds = list(model.ARM_SPEEDS)
        self.kinematics = model.arm_urdf(self.side).encode("utf-8")

    def _apply_side(self, side: str) -> None:
        self.side = model.side_name(side)
        self.kinematics = model.arm_urdf(self.side).encode("utf-8")
        self.positions = model.clamp_arm(self.side, self.positions)

    async def get_end_position(self, *, extra: dict[str, Any] | None = None, timeout: float | None = None, **kwargs) -> Pose:
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

    async def move_to_position(self, pose: Pose, *, extra: dict[str, Any] | None = None, timeout: float | None = None, **kwargs):
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

    async def get_manual_mode(self, *, extra: dict[str, Any] | None = None, timeout: float | None = None, **kwargs) -> bool:
        return False

    async def get_properties(self, *, extra: dict[str, Any] | None = None, timeout: float | None = None, **kwargs):
        # Newer SDKs require this. The response type is not in older wheels.
        from viam.proto.component.arm import GetPropertiesResponse

        return GetPropertiesResponse(support_manual_mode=False, support_cartesian_commands=False)

    async def stop(self, *, extra: dict[str, Any] | None = None, timeout: float | None = None, **kwargs):
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
        self._apply_side(_require_side(struct_to_dict(config.attributes)))


class OpenArm(_ArmBase, EasyResource):
    """7-DoF OpenArm 2.0 arm on a shared SocketCAN-FD bus."""

    MODEL: ClassVar[str] = "viam-labs:openarm-2:arm"  # type: ignore[assignment]

    def __init__(self, name: str):
        super().__init__(name)
        self.interface = ""
        self.gain_scale = 1.0
        self.kp = list(model.ARM_KP)
        self.kd = list(model.ARM_KD)
        self.session: CanSession | None = None
        self.send_ids = list(model.SEND_IDS[: model.ARM_JOINTS])

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
        self.interface = attributes["can_interface"]
        self.gain_scale = float(attributes.get("gain_scale", 1.0))
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

    async def get_end_position(self, *, extra: dict[str, Any] | None = None, timeout: float | None = None, **kwargs) -> Pose:
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
        session = self._session()
        session.prepare(self.send_ids)
        await wait_until(
            lambda: self._feedback_ready(),
            1.0,
            feedback_timeout(self.interface, session),
        )
        await wait_for_enable(
            self.interface,
            session,
            [(f"joint{index + 1}", send_id) for index, send_id in enumerate(self.send_ids)],
        )
        self._read_feedback()
        goal = model.clamp_arm(self.side, _radians(positions.values))
        operation = operation_from(self, kwargs)

        async def update(values):
            if not self.motion.moving:
                return
            self.positions = list(values)
            self._command(values)

        self.positions = await self.motion.run(self.positions, goal, self.command_speeds, update, operation)

    async def stop(self, *, extra: dict[str, Any] | None = None, timeout: float | None = None, **kwargs):
        await super().stop(extra=extra, timeout=timeout, **kwargs)
        if self.session is not None:
            self.session.hold(self.send_ids)

    def close_hardware(self) -> None:
        session = self.session
        self.session = None
        if session is None:
            return
        session.disable(self.send_ids)
        session.drop_motors(self.send_ids)
        release(session)

    async def close(self):
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
        self.positions = model.clamp_arm(self.side, model.kinematic_from_motor(self.side, motor_positions))

    def _command(self, kinematic: Sequence[float]) -> None:
        session = self._session()
        motor_positions = model.motor_from_kinematic(self.side, kinematic)
        for index, send_id in enumerate(self.send_ids):
            session.set_target(
                send_id,
                motor_positions[index],
                self.kp[index] * self.gain_scale,
                self.kd[index] * self.gain_scale,
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
