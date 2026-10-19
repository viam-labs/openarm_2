"""Machine fragments for a bimanual OpenArm 2.0 cell.

Both fragments use the same component names and frames. The hardware fragment
selects the CAN models. The fake fragment selects the in-memory models so a
motion client can be tested without a robot.
"""

from __future__ import annotations

import math

from openarm_viam.description import gripper_tcp_mm, head_stereo_mm, mount_mm

_IDENTITY = {"type": "ov_degrees", "value": {"x": 0, "y": 0, "z": 1, "th": 0}}
# URDF rpy is radians: pi, pi/2, pi. euler_angles in a fragment are radians too.
_GRASP_ORIENTATION = {
    "type": "euler_angles",
    "value": {"roll": math.pi, "pitch": math.pi / 2, "yaw": math.pi},
}
# The description uses +X forward and +Y left. Viam uses +Y forward, +Z up, and +X right.
# Quaternion for a +90 degree yaw. A bare 90 under euler_angles is 90 radians.
_HALF_TURN = math.sqrt(2) / 2
_ARM_ORIENTATION = {
    "type": "quaternion",
    "value": {"w": _HALF_TURN, "x": 0, "y": 0, "z": _HALF_TURN},
}
# Description camera_link +X looks forward. The optical frame looks along body +Y,
# with image right along body +X and image down along -Z.
_HEAD_ORIENTATION = {"type": "ov_degrees", "value": {"x": 0, "y": 1, "z": 0, "th": -90}}


def bimanual_fragment(*, fake: bool) -> dict:
    arm_model = "viam-labs:openarm-2:fake-arm" if fake else "viam-labs:openarm-2:arm"
    gripper_model = "viam-labs:openarm-2:fake-gripper" if fake else "viam-labs:openarm-2:gripper"
    tcp = _xyz(gripper_tcp_mm())
    head = _viam_xyz(head_stereo_mm())
    components = [
        _component(
            "body",
            "camera",
            "rdk:builtin:fake",
            frame=_frame(
                "world",
                {"x": 0, "y": 0, "z": 0},
                geometry={
                    "type": "box",
                    "x": 200,
                    "y": 250,
                    "z": 740,
                    "translation": {"x": 0, "y": 0, "z": 370},
                },
            ),
        ),
        _component(
            "head-stereo",
            "camera",
            "rdk:builtin:fake",
            frame=_frame(
                "body",
                head,
                orientation=_HEAD_ORIENTATION,
                # ZED Mini outline in the optical frame: X across the image, Y down, Z out.
                geometry={
                    "type": "box",
                    "x": 125,
                    "y": 31,
                    "z": 27,
                },
            ),
        ),
    ]
    for side, interface in (("right", "can0"), ("left", "can1")):
        arm = f"{side}-arm"
        gripper = f"{side}-gripper"
        arm_attributes: dict = {"side": side}
        gripper_attributes: dict = {
            "side": side,
            "arm": arm,
            "notes": (
                "If a plan fails because the gripper intersects the flange, allow that pair "
                "in the motion service collision specification."
            ),
        }
        if not fake:
            arm_attributes["can_interface"] = interface
            gripper_attributes["can_interface"] = interface
        components.append(
            _component(
                arm,
                "arm",
                arm_model,
                attributes=arm_attributes,
                frame=_frame("body", _viam_xyz(mount_mm(side)), orientation=_ARM_ORIENTATION),
            )
        )
        components.append(
            _component(
                gripper,
                "gripper",
                gripper_model,
                attributes=gripper_attributes,
                depends_on=[arm],
                frame=_frame(arm, tcp, orientation=_GRASP_ORIENTATION),
            )
        )
        components.append(
            _component(
                f"{side}-camera",
                "camera",
                "rdk:builtin:fake",
                frame=_frame(
                    gripper,
                    {"x": 0, "y": 0, "z": -40},
                    geometry={"type": "box", "x": 40, "y": 30, "z": 25},
                ),
            )
        )
    # This object is the fragment config itself. The Viam builder reads
    # top-level components with an api field and does not look inside a wrapper.
    return {
        "modules": [
            {
                "name": "openarm",
                "type": "registry",
                "module_id": "viam-labs:openarm-2",
                "version": "latest",
            }
        ],
        "components": components,
        "services": [
            {
                "name": "builtin",
                "api": "rdk:service:motion",
                "model": "rdk:builtin:builtin",
            }
        ],
    }


_APIS = {
    "arm": "rdk:component:arm",
    "camera": "rdk:component:camera",
    "gripper": "rdk:component:gripper",
}


def _component(
    name: str,
    subtype: str,
    model: str,
    *,
    attributes: dict | None = None,
    depends_on: list[str] | None = None,
    frame: dict | None = None,
) -> dict:
    component = {
        "name": name,
        "api": _APIS[subtype],
        "model": model,
        "attributes": attributes or {},
        "depends_on": depends_on or [],
    }
    if frame is not None:
        component["frame"] = frame
    return component


def _frame(
    parent: str,
    translation: dict,
    *,
    orientation: dict | None = None,
    geometry: dict | None = None,
) -> dict:
    frame = {
        "parent": parent,
        "translation": translation,
        "orientation": orientation or _IDENTITY,
    }
    if geometry is not None:
        frame["geometry"] = geometry
    return frame


def _xyz(values: tuple[float, float, float]) -> dict:
    return {"x": values[0], "y": values[1], "z": values[2]}


def _viam_xyz(openarm_mm: tuple[float, float, float]) -> dict:
    """Map a description pose into the Viam body frame.

    The description is +X forward and +Y left. Viam is +Y forward and +X right.
    """

    forward, left, up = openarm_mm
    return {"x": -left, "y": forward, "z": up}
