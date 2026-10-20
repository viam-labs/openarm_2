import asyncio
import math
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
        await arm.move_to_joint_positions(JointPositions(values=[0, 0, 0, 0, math.degrees(0.35), 0, 0]))
        elapsed = time.monotonic() - started
        positions = list((await arm.get_joint_positions()).values)
        assert positions[4] == pytest.approx(math.degrees(0.35))
        # Default wrist speed is 1.0 rad/s with cosine ease, so 0.35 rad takes ~0.55 s.
        assert elapsed >= 0.35

        fast = FakeArm.new(_config("fast-arm", {"side": "right", "speed_scale": 3.0}), {})
        started = time.monotonic()
        await fast.move_to_joint_positions(JointPositions(values=[0, 0, 0, 0, math.degrees(0.35), 0, 0]))
        assert time.monotonic() - started < 0.35

        await arm.move_to_joint_positions(JointPositions(values=[0, 0, 0, 900, 0, 0, 0]))
        clamped = list((await arm.get_joint_positions()).values)
        assert clamped[3] == pytest.approx(math.degrees(2.4435))

        pose = await arm.get_end_position()
        assert pose.y == pytest.approx(-122.5)
        kinematics = await arm.get_kinematics()
        assert kinematics[0] == 2
        assert b"openarm_right_joint1" in kinematics[1]

        await arm.move_through_joint_positions([])
        await arm.move_through_joint_positions([JointPositions(values=[0, 0, 0, 0, 0, 0, 0])])
        assert (await arm.get_manual_mode()) is False
        assert await arm.get_3d_models() == {}

    asyncio.run(run())


def test_stop_cancels_a_fake_arm_move():
    async def run():
        arm = FakeArm.new(_config("left-arm", {"side": "left"}), {})
        task = asyncio.create_task(
            arm.move_to_joint_positions(JointPositions(values=[math.degrees(1.0), 0, 0, 0, 0, 0, 0]))
        )
        await asyncio.sleep(0.05)
        await arm.stop()
        await task
        position = list((await arm.get_joint_positions()).values)[0]
        assert position < math.degrees(1.0)
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


def test_fake_arm_do_command_moves_to_neutral_and_powers_off():
    async def run():
        arm = FakeArm.new(_config("right-arm", {"side": "right", "speed_scale": 4.0}), {})
        await arm.move_to_joint_positions(
            JointPositions(values=[0, 0, 0, math.degrees(0.4), 0, 0, 0])
        )
        result = await arm.do_command({"command": "move_to_neutral", "speed_scale": 1.0})
        assert result["ok"] is True
        assert result["command"] == "move_to_neutral"
        assert list((await arm.get_joint_positions()).values) == pytest.approx([0, 0, 0, 0, 0, 0, 0])

        powered = await arm.do_command({"command": "power_off"})
        assert powered == {"ok": True, "command": "power_off"}
        assert await arm.is_moving() is False

        with pytest.raises(ValueError, match="unsupported DoCommand"):
            await arm.do_command({"command": "dance"})

    asyncio.run(run())


def test_fake_gripper_do_command_powers_off():
    async def run():
        gripper = FakeGripper.new(_config("right-gripper", {"side": "right"}), {})
        await gripper.open()
        result = await gripper.do_command({"command": "power_off"})
        assert result == {"ok": True, "command": "power_off"}
        assert await gripper.is_moving() is False

    asyncio.run(run())


def test_hardware_arm_power_off_disables_motors_without_closing():
    from openarm_viam.can.bus import CanSession, Motor
    from openarm_viam.models.arm import OpenArm

    async def run():
        arm = OpenArm("right-arm")
        arm.side = "right"
        session = CanSession("can0")
        session.motors = {
            send_id: Motor("DM4310", send_id, send_id + 0x10, mode="mit", measured=0.1)
            for send_id in arm.send_ids
        }
        sent: list[int] = []
        session._send = lambda packets: sent.extend(arbitration_id for arbitration_id, _data in packets)  # type: ignore[method-assign]
        arm.session = session
        arm._servo_goal = [0.1] * 7

        result = await arm.do_command({"command": "power_off"})
        assert result == {"ok": True, "command": "power_off"}
        assert arm.session is session
        assert arm._servo_goal is None
        assert sent == list(arm.send_ids)
        assert all(session.motors[send_id].mode == "idle" for send_id in arm.send_ids)

    asyncio.run(run())


def test_background_settle_is_cancelled_by_the_next_move():
    from openarm_viam.models.arm import OpenArm

    async def run():
        arm = OpenArm("right-arm")
        arm.side = "right"
        started = asyncio.Event()
        cancelled = asyncio.Event()

        async def slow_settle(goal, holding, token):
            started.set()
            try:
                await asyncio.sleep(30)
            except asyncio.CancelledError:
                cancelled.set()
                raise

        arm._settle_to_goal = slow_settle  # type: ignore[method-assign]
        arm._schedule_settle([0.0] * 7, [True] * 7)
        await asyncio.wait_for(started.wait(), timeout=1)
        assert arm._settle_task is not None and not arm._settle_task.done()

        # A follow-up schedule (what the next move does) must preempt settle.
        second_started = asyncio.Event()

        async def second_settle(goal, holding, token):
            second_started.set()
            await asyncio.sleep(0)

        arm._settle_to_goal = second_settle  # type: ignore[method-assign]
        arm._schedule_settle([0.1] * 7, [False] * 7)
        await asyncio.wait_for(cancelled.wait(), timeout=1)
        await asyncio.wait_for(second_started.wait(), timeout=1)

        arm._cancel_settle()

    asyncio.run(run())


def test_settle_skips_wait_when_already_on_target():
    from openarm_viam.can.bus import CanSession, Motor
    from openarm_viam.can.damiao import STATUS_ENABLED
    from openarm_viam.models.arm import OpenArm

    async def run():
        arm = OpenArm("right-arm")
        arm.side = "right"
        goal = [0.1, 0.0, 0.0, 0.5, 0.0, 0.0, 0.0]
        session = CanSession("can0")
        session.motors = {
            send_id: Motor(
                "DM4310",
                send_id,
                send_id + 0x10,
                mode="mit",
                measured=goal[index],
                error=STATUS_ENABLED,
                last_rx=time.monotonic(),
            )
            for index, send_id in enumerate(arm.send_ids)
        }
        arm.session = session
        arm.positions = list(goal)
        waited = False

        async def fail_if_wait(token):
            nonlocal waited
            waited = True
            raise AssertionError("settle should not wait when already on target")

        arm._wait_nearly_still = fail_if_wait  # type: ignore[method-assign]
        commands: list[list[float]] = []
        original_command = arm._command

        def track_command(kinematic, holding=None):
            commands.append(list(kinematic))
            original_command(kinematic, holding)

        arm._command = track_command  # type: ignore[method-assign]
        await arm._settle_to_goal(goal, [True] * 7, arm._settle_token)
        assert waited is False
        assert commands == [goal]

    asyncio.run(run())
