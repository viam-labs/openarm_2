# OpenArm 2 Viam module

Python Viam module for the OpenArm 2.0 arms and pinch grippers. It talks to the Damiao motors over SocketCAN-FD. It does not wrap ROS. The Viam motion service does inverse kinematics and collision checking from the URDF this module returns and from the frames in the fragments.

## Models

| Model | API | Role |
| --- | --- | --- |
| `viam-labs:openarm-2:arm` | `rdk:component:arm` | One 7-DoF arm on a CAN-FD interface |
| `viam-labs:openarm-2:gripper` | `rdk:component:gripper` | Pinch gripper on that same interface |
| `viam-labs:openarm-2:fake-arm` | `rdk:component:arm` | Same kinematics, positions kept in memory |
| `viam-labs:openarm-2:fake-gripper` | `rdk:component:gripper` | Same pinch model, no CAN socket |

`module_id` is `viam-labs:openarm-2`. The public registry page is [viam-labs:openarm-2](https://app.viam.com/module/viam-labs/openarm-2).

### Arm

Attributes:

| Attribute | Required | Description |
| --- | --- | --- |
| `side` | yes | `left` or `right` |
| `can_interface` | hardware only | SocketCAN name, usually `can0` for the right arm and `can1` for the left |
| `gain_scale` | no | Multiplier on the MIT gains. Default `1.0` |
| `kp`, `kd` | no | Seven MIT gains. Defaults are the OpenArm driver values |

Joint commands are URDF angles in radians. Motor positions are those angles plus the driver zero offsets. The first move enables the motors. `stop` cancels the trajectory and holds. Closing the component disables its motors.

`move_to_position` is not implemented. Send a pose goal through the motion service. The service reads `get_kinematics` and calls `move_to_joint_positions`.

### Gripper

The gripper uses motor `0x08` on the arm's CAN interface, so the arm and gripper in one module process share a single socket. Input `0` is closed and `1` is open.

| Attribute | Required | Description |
| --- | --- | --- |
| `side` | yes | `left` or `right` |
| `can_interface` | hardware only | Same interface as that side's arm |
| `arm` | no | Arm component name. Returned as a dependency so the arm starts first |
| `gain_scale` | no | Multiplier on the gripper MIT gains |

### Fake models

`fake-arm` and `fake-gripper` take `side` and no `can_interface`. They stream the same speed-limited joint path against memory, which is what the fake fragment uses.

## Fragments

`fragments/openarm-bimanual.json` is the hardware cell. `fragments/openarm-bimanual-fake.json` is the same component names and frames with the fake arm and gripper models. A motion client can keep using `right-arm` and `left-arm` when you switch fragments.

```text
world
├── body
│   └── head-stereo
├── right-arm
│   ├── right-gripper
│   └── right-camera
└── left-arm
    ├── left-gripper
    └── left-camera
```

Arm bases sit on the body at the v2 mount points: right `(0, -30, 697)` mm and left `(0, 30, 697)` mm. Gripper frames are the pinch center. `head-stereo` starts at the commented `zed_front_mount_point` from the v2 description, `(120, 0, 820)` mm on the body, tilted down 40 degrees. The body box stops below that camera. Treat the head pose as a starting value and correct it after one measurement.

`body`, `head-stereo`, `right-camera`, and `left-camera` are builtin fake cameras so the frames and collision boxes exist before real cameras are attached. Camera drivers are not part of this module.

- Point each in-hand camera at the builtin `webcam` model and set `video_path`.
- The top camera is a Stereolabs ZED Mini. `webcam` can show its side-by-side USB image. Depth needs a ZED SDK module, which is a separate project. Keep the component name `head-stereo` so the frame stays put.

If a plan fails because the gripper model intersects the flange, allow that pair in the motion service collision specification. The gripper attributes include that note.

## Run on the robot

Bring the interfaces up before viam-server starts the module:

```sh
sudo ip link set can0 up type can bitrate 1000000 dbitrate 5000000 fd on
sudo ip link set can1 up type can bitrate 1000000 dbitrate 5000000 fd on
```

The fragments install registry module `viam-labs:openarm-2`. For a local checkout, point the machine config at `run.sh` instead of the registry entry.

## Safety

Keep an emergency stop within reach, keep the workspace clear, and know how to cut power. Gains start from the published OpenArm driver defaults. Use `gain_scale` below `1` the first time you enable a new arm, then raise it only after a small move looks right. A large position step at full gain can produce a fast motion.

Motors stay disabled until the first joint move. That move waits for feedback and holds the measured pose as the start of the trajectory, so the arm is not commanded to the URDF zero just because the module started.

## Development

```sh
python3 -m pip install -r requirements-dev.txt
python3 -m pytest
```

The module entrypoint is `python3 -m openarm_viam` with `src` on `PYTHONPATH`, which is what `run.sh` does.
