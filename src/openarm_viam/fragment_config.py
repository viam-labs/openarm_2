"""Machine fragments for a bimanual OpenArm 2.0 cell.

Both fragments use the same component names and frames. The hardware fragment
selects the CAN models. The fake fragment selects the in-memory models so a
motion client can be tested without a robot.
"""

from __future__ import annotations

from openarm_viam.description import gripper_tcp_mm, head_stereo_mm, mount_mm

_IDENTITY = {"type": "ov_degrees", "value": {"x": 0, "y": 0, "z": 1, "th": 0}}
_GRASP_ORIENTATION = {"type": "euler_angles", "value": {"roll": 180, "pitch": 90, "yaw": 180}}
# Starting tilt from the commented zed_front_mount_point. Calibrate on hardware.
_HEAD_ORIENTATION = {"type": "ov_degrees", "value": {"x": 0, "y": 1, "z": 0, "th": -40}}


def bimanual_fragment(*, fake: bool) -> dict:
    arm_model = "viam-labs:openarm-2:fake-arm" if fake else "viam-labs:openarm-2:arm"
    gripper_model = "viam-labs:openarm-2:fake-gripper" if fake else "viam-labs:openarm-2:gripper"
    tcp = _xyz(gripper_tcp_mm())
    head = _xyz(head_stereo_mm())
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
                    "x": 250,
                    "y": 200,
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
                geometry={
                    "type": "box",
                    "x": 145,
                    "y": 55,
                    "z": 50,
                    "translation": {"x": 0, "y": 0, "z": -25},
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
                frame=_frame("body", _xyz(mount_mm(side))),
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
    return {
        "name": "openarm-bimanual-fake" if fake else "openarm-bimanual",
        "fragment": {
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
                    "namespace": "rdk",
                    "type": "motion",
                    "model": "rdk:builtin:builtin",
                }
            ],
        },
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
        "namespace": "rdk",
        "type": subtype,
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
