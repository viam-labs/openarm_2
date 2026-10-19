import json
from pathlib import Path

from openarm_viam.description import gripper_tcp_mm, head_stereo_mm, mount_mm
from openarm_viam.fragment_config import bimanual_fragment

ROOT = Path(__file__).parents[1] / "fragments"


def _load(name: str) -> dict:
    return json.loads((ROOT / name).read_text())


def test_fragment_files_match_the_builder():
    assert _load("openarm-bimanual.json") == bimanual_fragment(fake=False)
    assert _load("openarm-bimanual-fake.json") == bimanual_fragment(fake=True)


def test_fragments_share_names_and_frames():
    hardware = _load("openarm-bimanual.json")
    fake = _load("openarm-bimanual-fake.json")
    hardware_components = {component["name"]: component for component in hardware["components"]}
    fake_components = {component["name"]: component for component in fake["components"]}
    assert set(hardware_components) == set(fake_components)
    assert set(hardware_components) == {
        "body",
        "head-stereo",
        "right-arm",
        "right-gripper",
        "right-camera",
        "left-arm",
        "left-gripper",
        "left-camera",
    }
    for name in hardware_components:
        assert hardware_components[name]["frame"] == fake_components[name]["frame"]

    assert hardware["services"] == fake["services"]
    assert hardware["services"][0]["api"] == "rdk:service:motion"
    assert hardware_components["body"]["api"] == "rdk:component:camera"
    assert "type" not in hardware_components["right-arm"]
    assert "can_interface" not in fake_components["right-arm"]["attributes"]
    assert hardware_components["right-arm"]["attributes"]["can_interface"] == "can0"
    assert hardware_components["left-arm"]["attributes"]["can_interface"] == "can1"
    assert hardware_components["right-arm"]["model"] == "viam-labs:openarm-2:arm"
    assert fake_components["right-arm"]["model"] == "viam-labs:openarm-2:fake-arm"
    assert fake_components["left-gripper"]["model"] == "viam-labs:openarm-2:fake-gripper"

    right_mount = mount_mm("right")
    left_mount = mount_mm("left")
    tcp = gripper_tcp_mm()
    head = head_stereo_mm()
    assert hardware_components["right-arm"]["frame"]["parent"] == "body"
    assert hardware_components["right-arm"]["frame"]["translation"] == {"x": right_mount[0], "y": right_mount[1], "z": right_mount[2]}
    assert hardware_components["left-arm"]["frame"]["translation"]["y"] == left_mount[1]
    assert hardware_components["head-stereo"]["frame"]["parent"] == "body"
    assert hardware_components["head-stereo"]["frame"]["translation"] == {"x": head[0], "y": head[1], "z": head[2]}
    assert hardware_components["head-stereo"]["frame"]["geometry"]["type"] == "box"
    assert hardware_components["right-gripper"]["frame"]["parent"] == "right-arm"
    assert hardware_components["right-gripper"]["frame"]["translation"] == {"x": tcp[0], "y": tcp[1], "z": tcp[2]}
    assert hardware_components["right-camera"]["frame"]["parent"] == "right-gripper"
    assert hardware_components["left-camera"]["frame"]["parent"] == "left-gripper"
    assert hardware_components["body"]["frame"]["geometry"]["translation"]["z"] + hardware_components["body"]["frame"]["geometry"]["z"] / 2 < head[2]
