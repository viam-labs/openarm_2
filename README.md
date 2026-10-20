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
| `speed_scale` | no | Multiplier on the default joint streaming speeds. Default `1.0`. Try `1.5`–`2.0` if jogs feel slow |
| `gain_scale` | no | Multiplier on the MIT gains. Default `1.0`. Lower this (for example `0.7`) if motors buzz |
| `hold_gain_scale` | no | Extra multiplier for joints that are not changing in the current move. Default `1.15` |
| `settle_gain` | no | Optional still-gated bias trim on held joints that have drooped by more than ~1.5°. Default `0` (off). Max `1.0` |
| `settle_passes` | no | Max still-sampled corrections when `settle_gain` is enabled. Default `8` |
| `kp`, `kd` | no | Seven MIT gains. Defaults balance gravity hold against buzzing |

Joint commands use Viam's degree units. The module converts those to the URDF's radians. After OpenArm's firmware zero-position calibration, motor angles match the URDF, so no extra joint offsets are applied. Moves use a cosine ease-in/out. Each move starts from the last commanded pose, not the encoder reading. A PD-held joint sits slightly below its target under gravity, so restarting from the measured angle would remove its holding torque and let it fall. A requested joint counts as held when it matches the last command or echoes the current reading within about 0.3° (jog clients send `get_joint_positions` back for joints they are not changing). Held joints keep their last command. A deliberate change smaller than 0.3° on a joint is treated as a hold. Optional `settle_gain` can trim large held-joint droop in the background; it is off by default. A new move, `stop`, or `power_off` cancels any in-flight settle. If a pose buzzes, lower `gain_scale` or `hold_gain_scale`. The first move clears latched motor faults and enables the motors. If a motor does not report enabled within two seconds, the move fails and lists each joint's state, for example `joint3=undervoltage`. `stop` cancels the trajectory and holds. Closing the component disables its motors.

`move_to_position` is not implemented. Send a pose goal through the motion service. The service reads `get_kinematics` and calls `move_to_joint_positions`.

DoCommands:

| Command | Args | Description |
| --- | --- | --- |
| `move_to_neutral` | optional `speed_scale` | Joint-space move to URDF zero (all joints `0`). Default speed is `0.35`× the configured joint speeds. Not collision-checked by the motion service |
| `power_off` | none | Cancel any move and disable this arm's motors. The next joint command re-enables them |

```json
{"command": "move_to_neutral"}
{"command": "move_to_neutral", "speed_scale": 0.2}
{"command": "power_off"}
```

Call `power_off` on the gripper as well if that motor should also torque-off.

### Gripper

The gripper uses motor `0x08` (a DM3507) on the arm's CAN interface, so the arm and gripper in one module process share a single socket. Input `0` is closed and `1` is open. Like the OpenArm driver, it runs in position-force mode with a speed and torque limit, so it stops on an object instead of pushing at full stiffness.

| Attribute | Required | Description |
| --- | --- | --- |
| `side` | yes | `left` or `right` |
| `can_interface` | hardware only | Same interface as that side's arm |
| `arm` | no | Arm component name. Returned as a dependency so the arm starts first |

DoCommand `power_off` cancels any move and disables the gripper motor. The next open/grab re-enables it.

### Fake models

`fake-arm` and `fake-gripper` take `side` and no `can_interface`. They stream the same speed-limited joint path against memory, which is what the fake fragment uses.

## Fragments

`fragments/openarm-bimanual.json` is the hardware cell. `fragments/openarm-bimanual-fake.json` is the same component names and frames with the fake arm and gripper models. A motion client can keep using `right-arm` and `left-arm` when you switch fragments.

Each file is the fragment config. In the Viam fragment JSON editor, replace the document with the whole file. The builder lists a block only when `components` is at the top level and each component has an `api`.

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

The body frame uses the Viam convention: `+Y` forward, `+Z` up, and `+X` right. The description's mounts are `+X` forward and `+Y` left, so each arm is yawed 90 degrees where it meets the body. In that frame the right arm base is `(30, 0, 697)` mm and the left arm base is `(-30, 0, 697)` mm. Gripper frames are the pinch center, still expressed in the arm flange. `head-stereo` is the commented `zed_front_mount_point`, `(0, 120, 820)` mm, looking forward. The 125 mm bar lies across the body. The body box stops below that camera.

`body`, `head-stereo`, `right-camera`, and `left-camera` are builtin fake cameras so the frames and collision boxes exist before real cameras are attached. Camera drivers are not part of this module.

- Point each in-hand camera at the builtin `webcam` model and set `video_path`.
- The top camera is a Stereolabs ZED Mini. `webcam` can show its side-by-side USB image. Depth needs a ZED SDK module, which is a separate project. Keep the component name `head-stereo` so the frame stays put.

If a plan fails because the gripper model intersects the flange, allow that pair in the motion service collision specification. The gripper attributes include that note.

## Run on the robot

Bring the interfaces up before viam-server starts the module:

```sh
sudo ip link set can0 up type can bitrate 1000000 dbitrate 5000000 fd on restart-ms 100
sudo ip link set can1 up type can bitrate 1000000 dbitrate 5000000 fd on restart-ms 100
```

The fragments install registry module `viam-labs:openarm-2`. For a local checkout, point the machine config at `run.sh` instead of the registry entry.

## Safety

Keep an emergency stop within reach, keep the workspace clear, and know how to cut power. Gains start from the published OpenArm driver defaults. Use `gain_scale` below `1` the first time you enable a new arm, then raise it only after a small move looks right. A large position step at full gain can produce a fast motion. Joints that are not part of a move are held at their measured start with `hold_gain_scale` so gravity does not pull them while another joint moves.

Motors stay disabled until the first joint or gripper command. That command enables them and holds the measured pose until the trajectory starts, so the arm is not commanded to the URDF zero just because the module started. If the arm is unplugged or powered off, the next move clears stale state, reopens the CAN socket if needed, and re-enables the motors — no module restart required. Once a joint is holding, later moves do not re-run CLEAR_ERROR/enable just because a feedback frame was late — that interrupt was dropping base-joint torque. CLEAR is reserved for latched faults; MIT holds keep streaming through re-enable. A motor LED that turns green and then red means it stopped hearing CAN. `restart-ms 100` brings the interface back after a bus-off.

## Development

```sh
python3 -m pip install -r requirements-dev.txt
python3 -m pytest
```

`meta.json` sets `first_run` to `setup.sh`, which installs `viam-sdk` and `python-can` into `.venv`. `run.sh` runs that setup again if `.venv` is missing, then starts `.venv/bin/python` with `src` on `PYTHONPATH`.
