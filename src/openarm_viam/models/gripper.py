"""OpenArm 2.0 pinch gripper.

The gripper motor shares the arm's CAN interface. ``get_current_inputs``
returns one value: 0 is closed and 1 is open, matching the gripper URDF.
"""

from __future__ import annotations

from typing import Any, ClassVar, Mapping, Sequence

from typing_extensions import Self
from viam.components.gripper import Gripper
from viam.operations import run_with_operation
from viam.proto.app.robot import ComponentConfig
from viam.proto.common import KinematicsFileFormat, ResourceName
from viam.resource.base import ResourceBase
from viam.resource.easy_resource import EasyResource
from viam.utils import ValueTypes, struct_to_dict

from openarm_viam import description as model
from openarm_viam.can.bus import CanSession, Motor, acquire, release
from openarm_viam.models.motion import Motion, operation_from, wait_until

_OPEN_INPUT = [model.GRIPPER_OPEN]
_CLOSED_INPUT = [model.GRIPPER_CLOSED]


class _GripperBase(Gripper):
    def __init__(self, name: str):
        super().__init__(name)
        self.side = "right"
        self.open_amount = model.GRIPPER_CLOSED
        self.motion = Motion()
        self.holding = False
        self.kinematics = model.gripper_urdf(self.side).encode("utf-8")

    def _apply_side(self, side: str) -> None:
        self.side = model.side_name(side)
        self.kinematics = model.gripper_urdf(self.side).encode("utf-8")

    async def get_current_inputs(
        self, *, extra: dict[str, Any] | None = None, timeout: float | None = None, **kwargs
    ) -> list[float]:
        return [self.open_amount]

    async def get_kinematics(self, *, extra: dict[str, Any] | None = None, timeout: float | None = None, **kwargs):
        return (KinematicsFileFormat.KINEMATICS_FILE_FORMAT_URDF, self.kinematics)

    async def is_moving(self) -> bool:
        return self.motion.moving

    async def is_holding_something(
        self, *, extra: dict[str, Any] | None = None, timeout: float | None = None, **kwargs
    ) -> Gripper.HoldingStatus:
        return Gripper.HoldingStatus(is_holding_something=self.holding, meta={"open_amount": self.open_amount})

    async def open(self, *, extra: dict[str, Any] | None = None, timeout: float | None = None, **kwargs):
        await self.go_to_inputs(_OPEN_INPUT, extra=extra, timeout=timeout, **kwargs)
        self.holding = False

    async def grab(self, *, extra: dict[str, Any] | None = None, timeout: float | None = None, **kwargs) -> bool:
        await self.go_to_inputs(_CLOSED_INPUT, extra=extra, timeout=timeout, **kwargs)
        # A real object stops the fingers before they reach closed. The fake model reaches closed.
        self.holding = self.open_amount > 0.15
        return self.holding

    @run_with_operation
    async def go_to_inputs(
        self, values: list[float], *, extra: dict[str, Any] | None = None, timeout: float | None = None, **kwargs
    ):
        if not values:
            raise ValueError("gripper expects one input, 0 closed and 1 open")
        goal = min(model.GRIPPER_OPEN, max(model.GRIPPER_CLOSED, float(values[0])))
        operation = operation_from(self, kwargs)

        async def update(commanded):
            self.open_amount = commanded[0]

        await self.motion.run([self.open_amount], [goal], [model.GRIPPER_SPEED], update, operation)

    async def stop(self, *, extra: dict[str, Any] | None = None, timeout: float | None = None, **kwargs):
        self.motion.cancel()

    async def get_status(self, *, timeout: float | None = None, **kwargs) -> Mapping[str, ValueTypes]:
        return {"side": self.side, "open_amount": self.open_amount, "holding": self.holding, "moving": self.motion.moving}


class FakeGripper(_GripperBase, EasyResource):
    MODEL: ClassVar[str] = "viam-labs:openarm-2:fake-gripper"  # type: ignore[assignment]

    @classmethod
    def new(cls, config: ComponentConfig, dependencies: Mapping[ResourceName, ResourceBase]) -> Self:
        gripper = cls(config.name)
        gripper.reconfigure(config, dependencies)
        return gripper

    @classmethod
    def validate_config(cls, config: ComponentConfig) -> tuple[Sequence[str], Sequence[str]]:
        _require_side(struct_to_dict(config.attributes))
        return [], []

    def reconfigure(self, config: ComponentConfig, dependencies: Mapping[ResourceName, ResourceBase]):
        self._apply_side(_require_side(struct_to_dict(config.attributes)))


class OpenGripper(_GripperBase, EasyResource):
    MODEL: ClassVar[str] = "viam-labs:openarm-2:gripper"  # type: ignore[assignment]

    def __init__(self, name: str):
        super().__init__(name)
        self.interface = ""
        self.gain_scale = 1.0
        self.session: CanSession | None = None
        self.send_id = model.SEND_IDS[model.ARM_JOINTS]

    @classmethod
    def new(cls, config: ComponentConfig, dependencies: Mapping[ResourceName, ResourceBase]) -> Self:
        gripper = cls(config.name)
        gripper.reconfigure(config, dependencies)
        return gripper

    @classmethod
    def validate_config(cls, config: ComponentConfig) -> tuple[Sequence[str], Sequence[str]]:
        attributes = _hardware_attributes(struct_to_dict(config.attributes))
        arm_name = attributes.get("arm")
        if isinstance(arm_name, str) and arm_name:
            return [arm_name], []
        return [], []

    def reconfigure(self, config: ComponentConfig, dependencies: Mapping[ResourceName, ResourceBase]):
        self.close_hardware()
        attributes = _hardware_attributes(struct_to_dict(config.attributes))
        self._apply_side(attributes["side"])
        self.interface = attributes["can_interface"]
        self.gain_scale = float(attributes.get("gain_scale", 1.0))
        self.session = acquire(self.interface)
        self.session.add_motor(
            Motor(
                motor_type=model.MOTOR_TYPES[model.ARM_JOINTS],
                send_id=self.send_id,
                recv_id=model.RECV_IDS[model.ARM_JOINTS],
            )
        )
        self.logger.info("OpenArm gripper %s using %s side %s", self.name, self.interface, self.side)

    async def get_current_inputs(
        self, *, extra: dict[str, Any] | None = None, timeout: float | None = None, **kwargs
    ) -> list[float]:
        self._read_feedback()
        return await super().get_current_inputs(extra=extra, timeout=timeout, **kwargs)

    @run_with_operation
    async def go_to_inputs(
        self, values: list[float], *, extra: dict[str, Any] | None = None, timeout: float | None = None, **kwargs
    ):
        if not values:
            raise ValueError("gripper expects one input, 0 closed and 1 open")
        session = self._session()
        await wait_until(
            lambda: session.measured(self.send_id) is not None,
            1.0,
            f"timed out waiting for gripper feedback on {self.interface}",
        )
        self._read_feedback()
        goal = min(model.GRIPPER_OPEN, max(model.GRIPPER_CLOSED, float(values[0])))
        operation = operation_from(self, kwargs)

        async def update(commanded):
            if not self.motion.moving:
                return
            self.open_amount = commanded[0]
            session.set_target(
                self.send_id,
                model.gripper_motor(self.side, self.open_amount),
                model.GRIPPER_KP * self.gain_scale,
                model.GRIPPER_KD * self.gain_scale,
            )

        await self.motion.run([self.open_amount], [goal], [model.GRIPPER_SPEED], update, operation)
        self._read_feedback()

    async def stop(self, *, extra: dict[str, Any] | None = None, timeout: float | None = None, **kwargs):
        await super().stop(extra=extra, timeout=timeout, **kwargs)
        if self.session is not None:
            self.session.hold([self.send_id])

    def close_hardware(self) -> None:
        session = self.session
        self.session = None
        if session is None:
            return
        session.disable([self.send_id])
        session.drop_motors([self.send_id])
        release(session)

    async def close(self):
        self.motion.cancel()
        self.close_hardware()

    def _session(self) -> CanSession:
        if self.session is None:
            raise RuntimeError("gripper is not connected to a CAN interface")
        return self.session

    def _read_feedback(self) -> None:
        session = self.session
        if session is None:
            return
        measured = session.measured(self.send_id)
        if measured is None:
            return
        self.open_amount = model.gripper_open_amount(self.side, measured)


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
