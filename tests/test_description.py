import xml.etree.ElementTree as ET

import pytest

from openarm_viam import description as model


def test_motor_offset_round_trip():
    kinematic = [0.2, 0.1, -0.4, 1.2, 0.3, -0.2, 0.5]
    for side in ("left", "right"):
        motor = model.motor_from_kinematic(side, kinematic)
        recovered = model.kinematic_from_motor(side, motor)
        assert recovered == pytest.approx(kinematic)


def test_joint_limits_are_clamped_and_mirrored():
    clamped = model.clamp_arm("right", [9, -9, 0, 9, 0, 0, 0])
    assert clamped[0] == model.arm_joints("right")[0].upper
    assert clamped[1] == model.arm_joints("right")[1].lower
    assert clamped[3] == model.arm_joints("right")[3].upper

    right = model.arm_joints("right")[0]
    left = model.arm_joints("left")[0]
    assert left.lower == -right.upper
    assert left.upper == -right.lower
    assert left.axis[1] == -right.axis[1]


def test_gripper_open_amount_round_trip():
    for side in ("left", "right"):
        for amount in (0.0, 0.25, 1.0):
            motor = model.gripper_motor(side, amount)
            assert model.gripper_open_amount(side, motor) == amount
    assert model.gripper_motor("right", 1.0) < 0
    assert model.gripper_motor("left", 1.0) > 0


def test_arm_urdf_has_seven_joints_and_collision():
    for side in ("left", "right"):
        root = ET.fromstring(model.arm_urdf(side))
        joints = [joint for joint in root.findall("joint") if joint.get("type") == "revolute"]
        assert len(joints) == 7
        assert root.find("link/collision/geometry/cylinder") is not None
        limit = joints[0].find("limit")
        assert limit is not None
        assert float(limit.get("lower")) == model.arm_joints(side)[0].lower


def test_forward_kinematics_at_zero():
    right, _ = model.forward_kinematics("right", [0, 0, 0, 0, 0, 0, 0])
    left, orientation = model.forward_kinematics("left", [0, 0, 0, 0, 0, 0, 0])
    assert right[0] == pytest.approx(0)
    assert right[1] == pytest.approx(-122.5)
    assert right[2] == pytest.approx(-436.0)
    assert left[1] == pytest.approx(122.5)
    assert orientation[2] == 1
    assert orientation[3] == 0
