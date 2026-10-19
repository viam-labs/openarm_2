import asyncio
import time

import pytest
from viam.proto.app.robot import ComponentConfig
from viam.proto.component.arm import JointPositions
from viam.utils import dict_to_struct

from openarm_viam.models.arm import FakeArm
from openarm_viam.models.gripper import FakeGripper


def _config(name: str, attributes: dict) -> ComponentConfig:
    config = ComponentConfig(name=name)
    config.attributes.CopyFrom(dict_to_struct(attributes))
    return config


def test_fake_arm_streams_within_the_speed_cap_and_clamps():
    async def run():
        arm = FakeArm.new(_config("right-arm", {"side": "right"}), {})
        started = time.monotonic()
        await arm.move_to_joint_positions(JointPositions(values=[0, 0, 0, 0, 0.35, 0, 0]))
        elapsed = time.monotonic() - started
        positions = list((await arm.get_joint_positions()).values)
        assert positions[4] == pytest.approx(0.35)
        assert elapsed >= 0.08

        await arm.move_to_joint_positions(JointPositions(values=[0, 0, 0, 9, 0, 0, 0]))
        clamped = list((await arm.get_joint_positions()).values)
        assert clamped[3] == pytest.approx(2.4435)

        pose = await arm.get_end_position()
        assert pose.y == pytest.approx(-122.5)
        kinematics = await arm.get_kinematics()
        assert kinematics[0] == 2
        assert b"openarm_right_joint1" in kinematics[1]

    asyncio.run(run())


def test_stop_cancels_a_fake_arm_move():
    async def run():
        arm = FakeArm.new(_config("left-arm", {"side": "left"}), {})
        task = asyncio.create_task(arm.move_to_joint_positions(JointPositions(values=[1.0, 0, 0, 0, 0, 0, 0])))
        await asyncio.sleep(0.05)
        await arm.stop()
        await task
        position = list((await arm.get_joint_positions()).values)[0]
        assert position < 1.0
        assert await arm.is_moving() is False

    asyncio.run(run())


def test_fake_gripper_opens_and_closes():
    async def run():
        gripper = FakeGripper.new(_config("right-gripper", {"side": "right"}), {})
        await gripper.open()
        assert (await gripper.get_current_inputs()) == [1]
        held = await gripper.grab()
        assert held is False
        assert (await gripper.get_current_inputs()) == [0]
        assert (await gripper.is_holding_something()).is_holding_something is False

    asyncio.run(run())


def test_hardware_config_requires_a_can_interface():
    with pytest.raises(ValueError, match="can_interface"):
        from openarm_viam.models.arm import OpenArm

        OpenArm.validate_config(_config("right-arm", {"side": "right"}))
