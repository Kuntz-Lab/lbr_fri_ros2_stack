# lbr_fri_ros2_stack (Kuntz Lab fork)
[![License](https://img.shields.io/github/license/lbr-stack/lbr_fri_ros2_stack)](https://github.com/lbr-stack/lbr_fri_ros2_stack/tree/humble?tab=Apache-2.0-1-ov-file#readme) 
[![Documentation Status](https://readthedocs.org/projects/lbr-stack/badge/?version=latest)](https://lbr-stack.readthedocs.io/en/latest/?badge=latest)
[![JOSS](https://joss.theoj.org/papers/c43c82bed833c02503dd47f2637192ef/status.svg)](https://joss.theoj.org/papers/c43c82bed833c02503dd47f2637192ef) 
[![Code Style: Black](https://img.shields.io/badge/code%20style-black-000000.svg)](https://github.com/psf/black)

ROS 2 packages for the KUKA LBR, including communication to the real robot via the Fast Robot Interface ([FRI](https://github.com/lbr-stack/fri)) and [Gazebo](http://gazebosim.org/) simulation support. Included are the `iiwa7`, `iiwa14`, `med7`, and `med14`.

This is the **Kuntz Lab fork** of [lbr-stack/lbr_fri_ros2_stack](https://github.com/lbr-stack/lbr_fri_ros2_stack). On top of upstream it adds two lab robot configurations (`med14_tc`, `med14_robotiq_2f`), a MoveIt wrapper package with action-server based motion primitives (`lbr_motion`), the action definitions those servers use (`lbr_interfaces`), combined LBR + Robotiq gripper control, and launch files that bring all of it up in mock or hardware mode. See [What this fork adds](#what-this-fork-adds).

<body>
    <table>
        <tr>
            <th align="left" width="25%">LBR IIWA 7 R800</th>
            <th align="left" width="25%">LBR IIWA 14 R820</th>
            <th align="left" width="25%">LBR Med 7 R800</th>
            <th align="left" width="25%">LBR Med 14 R820</th>
        </tr>
        <tr>
            <td align="center"><img src="https://raw.githubusercontent.com/lbr-stack/lbr_fri_ros2_stack/humble/lbr_fri_ros2_stack/doc/img/foxglove/iiwa7_r800.png" alt="LBR IIWA 7 R800"></td>
            <td align="center"><img src="https://raw.githubusercontent.com/lbr-stack/lbr_fri_ros2_stack/humble/lbr_fri_ros2_stack/doc/img/foxglove/iiwa14_r820.png" alt="LBR IIWA 14 R820"></td>
            <td align="center"><img src="https://raw.githubusercontent.com/lbr-stack/lbr_fri_ros2_stack/humble/lbr_fri_ros2_stack/doc/img/foxglove/med7_r800.png" alt="LBR Med 7 R800"></td>
            <td align="center"><img src="https://raw.githubusercontent.com/lbr-stack/lbr_fri_ros2_stack/humble/lbr_fri_ros2_stack/doc/img/foxglove/med14_r820.png" alt="LBR Med 14 R820"></td>
        </tr>
    </table>
</body>

## Contents
- [Status](#status)
- [What this fork adds](#what-this-fork-adds)
- [Package overview](#package-overview)
- [Robot models](#robot-models)
- [Installation](#installation)
- [Quick start (mock)](#quick-start-mock)
- [Launch file reference](#launch-file-reference)
- [Hardware bringup (two-PC setup)](#hardware-bringup-two-pc-setup)
- [Motion API (`lbr_motion` + `lbr_interfaces`)](#motion-api-lbr_motion--lbr_interfaces)
- [Gepetto hand bringup](#gepetto-hand-bringup)
- [Servoing](#servoing)
- [Gripper control](#gripper-control)
- [3D sensors / octomap collision avoidance](#3d-sensors--octomap-collision-avoidance)
- [Which controller do I need?](#which-controller-do-i-need)
- [Troubleshooting](#troubleshooting)
- [Documentation](#documentation)
- [Citation](#citation)
- [Acknowledgements](#acknowledgements)

## Status
| OS             | ROS Distribution | FRI Version |  Build Status |
| :------------- | :--------------- | :---------- |  :----------- |
| `Ubuntu-22.04` | `humble`         | `1.11`      |  [![build-ubuntu-22.04-fri-1.11](https://github.com/lbr-stack/lbr_fri_ros2_stack/actions/workflows/build-ubuntu-22.04-fri-1.11.yml/badge.svg)](https://github.com/lbr-stack/lbr_fri_ros2_stack/actions/workflows/build-ubuntu-22.04-fri-1.11.yml) |
| `Ubuntu-22.04` | `humble`         | `1.14`      |  [![build-ubuntu-22.04-fri-1.14](https://github.com/lbr-stack/lbr_fri_ros2_stack/actions/workflows/build-ubuntu-22.04-fri-1.14.yml/badge.svg)](https://github.com/lbr-stack/lbr_fri_ros2_stack/actions/workflows/build-ubuntu-22.04-fri-1.14.yml) |
| `Ubuntu-22.04` | `humble`         | `1.15`      |  [![build-ubuntu-22.04-fri-1.15](https://github.com/lbr-stack/lbr_fri_ros2_stack/actions/workflows/build-ubuntu-22.04-fri-1.15.yml/badge.svg)](https://github.com/lbr-stack/lbr_fri_ros2_stack/actions/workflows/build-ubuntu-22.04-fri-1.15.yml) |
| `Ubuntu-22.04` | `humble`         | `1.16`      |  [![build-ubuntu-22.04-fri-1.16](https://github.com/lbr-stack/lbr_fri_ros2_stack/actions/workflows/build-ubuntu-22.04-fri-1.16.yml/badge.svg)](https://github.com/lbr-stack/lbr_fri_ros2_stack/actions/workflows/build-ubuntu-22.04-fri-1.16.yml) |
| `Ubuntu-22.04` | `humble`         | `2.5`      |  [![build-ubuntu-22.04-fri-2.5](https://github.com/lbr-stack/lbr_fri_ros2_stack/actions/workflows/build-ubuntu-22.04-fri-2.5.yml/badge.svg)](https://github.com/lbr-stack/lbr_fri_ros2_stack/actions/workflows/build-ubuntu-22.04-fri-2.5.yml) |
| `Ubuntu-22.04` | `humble`         | `2.7`      |  [![build-ubuntu-22.04-fri-2.7](https://github.com/lbr-stack/lbr_fri_ros2_stack/actions/workflows/build-ubuntu-22.04-fri-2.7.yml/badge.svg)](https://github.com/lbr-stack/lbr_fri_ros2_stack/actions/workflows/build-ubuntu-22.04-fri-2.7.yml) |

> [!IMPORTANT]
> This fork sets the FRI client SDK version in [lbr_description/ros2_control/lbr_system_config.yaml](lbr_description/ros2_control/lbr_system_config.yaml) to **2.6**. That value controls which state interfaces `lbr_system_interface.xacro` generates, so it must match the FRI client you build against (`FRI_CLIENT_VERSION` / the `fri` branch in your workspace). If your `fri` checkout is on a different branch (e.g. `fri-2.7`), update `major_version` / `minor_version` accordingly.

## What this fork adds

| Area | Addition |
| :--- | :--- |
| New packages | [`lbr_motion`](lbr_motion/) — MoveIt wrapper action servers, example clients, and servo command publishers.<br>[`lbr_interfaces`](lbr_interfaces/) — `MoveToPose` and `MoveHome` action definitions. |
| New robot models | [`med14_tc`](lbr_description/urdf/med14_tc/) — med14 + tendon-cube end effector + table.<br>[`med14_robotiq_2f`](lbr_description/urdf/med14_robotiq_2f/) — med14 + adapter + Robotiq 2F-140 gripper + table.<br>[`med14_gepetto`](lbr_description/urdf/med14_gepetto/) — med14 + table + the Gepetto hand's wrist frame at the fixed flange↔wrist mount offset. |
| New MoveIt configs | [`med14_tc_moveit_config`](lbr_moveit_config/med14_tc_moveit_config/), [`med14_robotiq_2f_moveit_config`](lbr_moveit_config/med14_robotiq_2f_moveit_config/) (the latter with an `arm` and a `gripper` planning group), [`med14_gepetto_moveit_config`](lbr_moveit_config/med14_gepetto_moveit_config/). |
| Combined control | [`lbr_description/ros2_control/combined_controllers.yaml`](lbr_description/ros2_control/combined_controllers.yaml) — LBR controllers plus `robotiq_gripper_controller` and `robotiq_activation_controller` under one controller manager. |
| New launch files | [`lbr_move_to_pose.launch.py`](lbr_bringup/launch/lbr_move_to_pose.launch.py), [`lbr_and_robotiq_hardware.launch.py`](lbr_bringup/launch/lbr_and_robotiq_hardware.launch.py), [`lbr_servoing.launch.py`](lbr_bringup/launch/lbr_servoing.launch.py), [`lbr_servoing_and_robotiq.launch.py`](lbr_bringup/launch/lbr_servoing_and_robotiq.launch.py), [`lbr_gepetto.launch.py`](lbr_bringup/launch/lbr_gepetto.launch.py). |
| Gepetto hand integration | `med14_gepetto` carries the `gepetto_core` mount transform as a real URDF frame, and `lbr_gepetto.launch.py` brings the arm, RViz, and the [`gepetto_ros`](../gepetto_ros) hand nodes up together. See [Gepetto hand bringup](#gepetto-hand-bringup). |
| Modified upstream files | `description.py` (passes `com_port` / `use_fake_hardware` to xacro, adds the new models to `model` choices), `moveit.py` + `move_group.launch.py` (`sensors_3d` / `sensors_3d_config` args and octomap params), `hardware.launch.py` and `mock.launch.py` (gripper-related args), `lbr_system_config.yaml` (FRI 2.6). |

## Package overview

| Package | Role |
| :--- | :--- |
| [lbr_bringup](lbr_bringup/) | Launch files and launch mixins for mock, Gazebo, and hardware bringup, MoveIt, MoveIt Servo, and RViz. |
| [lbr_description](lbr_description/) | URDF/xacro for all robots plus `ros2_control` controller configurations and the FRI system config. |
| [lbr_ros2_control](lbr_ros2_control/) | `ros2_control` hardware interface for the FRI, plus LBR-specific broadcasters and controllers. |
| [lbr_fri_ros2](lbr_fri_ros2/) | The FRI ↔ ROS 2 bridge (async client, filters, command guards). |
| [lbr_moveit_config](lbr_moveit_config/) | Per-model MoveIt configuration packages. |
| [lbr_demos](lbr_demos/) | Upstream C++/Python demos for the various command interfaces. |
| **[lbr_motion](lbr_motion/)** | **Fork:** action servers wrapping MoveIt (`move_to_pose`, `move_to_home`), example action clients, servo command publishers, and a TF end-effector pose listener. |
| **[lbr_interfaces](lbr_interfaces/)** | **Fork:** `MoveToPose.action` and `MoveHome.action`. |

## Robot models

| `model` | Description | Base frame | End-effector frame | MoveIt planning groups |
| :--- | :--- | :--- | :--- | :--- |
| `iiwa7`, `iiwa14`, `med7`, `med14` | Upstream bare arms | `lbr_link_0` | `lbr_link_ee` | `arm` |
| `med14_tc` | med14 with tendon-cube end effector and table | `lbr_link_0` | `lbr_tendon_robot_link` | `arm` |
| `med14_robotiq_2f` | med14 with Robotiq 2F-140 gripper, adapter, and table | `lbr_floating_link` | `lbr_robotiq_140_base_link` | `arm`, `gripper` |
| `med14_gepetto` | med14 with the Gepetto hand's wrist frame and table | `lbr_link_0` | `lbr_gepetto_wrist_link` | `arm` |

The base/end-effector frames above are the ones the `lbr_motion` servers use for TF lookups (see [move_to_pose_server.py](lbr_motion/lbr_motion/move_to_pose_server.py)); MoveIt itself plans the `arm` chain from `lbr_link_0` to `lbr_link_ee` in all three configs.

All three new models attach a table under the robot base so the table is part of the collision scene. The Robotiq model additionally defines an adapter plate between `lbr_link_ee` and the gripper.

> [!NOTE]
> `lbr_motion`'s servers only branch on `med14`, `med14_tc`, and `med14_robotiq_2f`; passing `robot_name:=med14_gepetto` falls through to a bare 7-DoF configuration and logs a warning. That is harmless for servoing, which does not use those servers.

## Installation

1. Install ROS 2 development tools

    ```shell
    sudo apt install ros-dev-tools
    ```

2. Create a workspace, clone, and install dependencies

    ```shell
    source /opt/ros/humble/setup.bash
    export FRI_CLIENT_VERSION=1.15
    mkdir -p lbr-stack/src && cd lbr-stack
    vcs import src --input https://raw.githubusercontent.com/lbr-stack/lbr_fri_ros2_stack/humble/lbr_fri_ros2_stack/repos-fri-${FRI_CLIENT_VERSION}.yaml
    rosdep install --from-paths src -i -r -y
    ```

    Then replace the upstream `lbr_fri_ros2_stack` in `src` with this fork:

    ```shell
    rm -rf src/lbr_fri_ros2_stack
    git clone git@github.com:Kuntz-Lab/lbr_fri_ros2_stack.git src/lbr_fri_ros2_stack
    ```

> [!NOTE]
> The FRI client is cloned from [fri](https://github.com/lbr-stack/fri) and must be available as a branch, refer to its [README](https://github.com/lbr-stack/fri?tab=readme-ov-file#contributing). Keep `FRI_CLIENT_VERSION` consistent with `lbr_description/ros2_control/lbr_system_config.yaml`.

3. **Extra dependency for `med14_robotiq_2f`.** The Robotiq model pulls in `robotiq_description` (xacro macro `robotiq_2f_140_macro.urdf.xacro`) and `robotiq_controllers` (`robotiq_controllers/RobotiqActivationController`) from [PickNikRobotics/ros2_robotiq_gripper](https://github.com/PickNikRobotics/ros2_robotiq_gripper), plus its serial dependency. Clone it into the same workspace:

    ```shell
    git clone -b humble https://github.com/PickNikRobotics/ros2_robotiq_gripper.git src/ros2_robotiq_gripper
    git clone -b ros2 https://github.com/tylerjw/serial.git src/serial
    rosdep install --from-paths src -i -r -y
    ```

    Without these packages, `med14_robotiq_2f` will fail at xacro parsing / controller loading time. `med14_tc` and the upstream models do not need them.

4. Build

    ```shell
    colcon build --symlink-install
    ```

## Quick start (mock)

Bare arm, no MoveIt:

1. Terminal 1 — mock robot and controllers:

    ```shell
    source install/setup.bash
    ros2 launch lbr_bringup mock.launch.py \
        model:=iiwa7 # [iiwa7, iiwa14, med7, med14, med14_tc, med14_robotiq_2f, med14_gepetto]
    ```

2. Terminal 2 — RViz:

    ```shell
    source install/setup.bash
    ros2 launch lbr_bringup rviz.launch.py \
        rviz_cfg_pkg:=lbr_bringup \
        rviz_cfg:=config/mock.rviz
    ```

> [!TIP]
> List all arguments of any launch file with `-s`, e.g. `ros2 launch lbr_bringup mock.launch.py -s`.

Full lab setup (robot + MoveIt + motion servers), all in one terminal:

```shell
ros2 launch lbr_bringup lbr_move_to_pose.launch.py model:=med14_robotiq_2f
```

Then, in a second terminal, command a pose:

```shell
ros2 run lbr_motion move_to_pose_client   # example client, edit the pose in main()
ros2 run lbr_motion move_home_client      # sends every arm joint to 0
```

Now, run the upstream [demos](https://lbr-stack.readthedocs.io/en/latest/lbr_fri_ros2_stack/lbr_demos/doc/lbr_demos.html). To get started with the real robot, check out the upstream [Hardware Setup](https://lbr-stack.readthedocs.io/en/latest/lbr_fri_ros2_stack/lbr_fri_ros2_stack/doc/hardware_setup.html) and [Hardware bringup](#hardware-bringup-two-pc-setup) below.

## Launch file reference

All launch files live in [lbr_bringup/launch](lbr_bringup/launch/). Fork additions are marked ★.

| Launch file | What it starts | Key arguments (default) |
| :--- | :--- | :--- |
| `mock.launch.py` | Robot description + `ros2_control` with mock hardware | `model` (`iiwa7`), `ctrl` (`joint_trajectory_controller`), `ctrl_cfg_pkg` (`lbr_description`), `ctrl_cfg` (`ros2_control/lbr_controllers.yaml`) |
| `gazebo.launch.py` | Gazebo simulation | `model`, `ctrl` |
| `hardware.launch.py` | FRI hardware interface + `ros2_control` | `model`, `robot_name` (`lbr`), `ctrl`, `ctrl_cfg`, ★`com_port` (`/dev/ttyUSB1`), ★`use_fake_hardware` (`false`) |
| `move_group.launch.py` | MoveIt `move_group` (+ RViz) | `model`, `mode` (`mock`), `rviz` (`true`), ★`sensors_3d` (`false`), ★`sensors_3d_config` (`''`) |
| `moveit_servo.launch.py` | MoveIt Servo node, auto-starts servo | `model`, `mode`, `default_enable_servo` |
| `rviz.launch.py` | RViz | `rviz_cfg_pkg`, `rviz_cfg` |
| ★`lbr_move_to_pose.launch.py` | Mock robot (in mock mode) + MoveIt + `move_to_pose_server` + `move_home_server` + gripper controllers | `model` (`med14_tc`), `mode` (`mock`), `rviz` (`true`), `sensors_3d` (`false`), `sensors_3d_config` (`''`) |
| ★`lbr_and_robotiq_hardware.launch.py` | `hardware.launch.py` + Robotiq gripper/activation controller spawners | `model` (`med14_robotiq_2f`), `robot_name` (`lbr`), `com_port` (`/dev/ttyUSB1`) |
| ★`lbr_servoing.launch.py` | Mock robot on `forward_position_controller` + MoveIt Servo + RViz | `model` (`med14_robotiq_2f`), `mode` (`mock`), `rviz` (`true`) |
| ★`lbr_servoing_and_robotiq.launch.py` | Same as above but also spawns the Robotiq controllers in mock mode | `model` (`med14_robotiq_2f`), `mode` (`mock`), `rviz` (`true`) |
| ★`lbr_gepetto.launch.py` | Mock robot + RViz + the Gepetto hand nodes; with `servo:=true`, MoveIt Servo on `forward_position_controller` | `model` (`med14_gepetto`), `mode` (`mock`), `servo` (`false`), `rviz` (`true`), `gepetto_nodes` (`sliders`), `moving_speed` (`0`), `torque_limit` (`0`), `command_hz` (`20.0`), `conda_prefix` (`~/miniconda3/envs/crest_py10`) |

Notes on the fork launch files:

- `lbr_move_to_pose.launch.py` and `lbr_servoing_and_robotiq.launch.py` automatically select `ros2_control/combined_controllers.yaml` when `model:=med14_robotiq_2f`, and `ros2_control/lbr_controllers.yaml` otherwise.
- In `mode:=hardware`, `lbr_move_to_pose.launch.py` deliberately does **not** start the robot description / controllers — those are expected to be running on the robot PC (see below). It only starts MoveIt and the motion servers.
- The Robotiq controller spawners in `lbr_move_to_pose.launch.py` are conditioned on `model == med14_robotiq_2f` **and** `mode in {mock, gazebo}`; in hardware mode `lbr_and_robotiq_hardware.launch.py` spawns them instead.

## Hardware bringup (two-PC setup)

The lab setup splits work across two machines on the same ROS 2 domain:

| PC | Requirements | Runs |
| :--- | :--- | :--- |
| **Robot PC** | Realtime / low-latency kernel, wired to the KUKA controller and to the Robotiq controller over USB | `lbr_and_robotiq_hardware.launch.py` (FRI hardware interface, `ros2_control`, gripper controllers) |
| **Perception PC** | No realtime kernel needed | `lbr_move_to_pose.launch.py mode:=hardware` (MoveIt, motion servers, RViz, cameras) |

1. On the robot PC (start the FRI application on the KUKA smartPAD first, as described in the upstream hardware setup docs):

    ```shell
    ros2 launch lbr_bringup lbr_and_robotiq_hardware.launch.py \
        model:=med14_robotiq_2f \
        com_port:=/dev/ttyUSB1
    ```

2. On the perception PC:

    ```shell
    ros2 launch lbr_bringup lbr_move_to_pose.launch.py \
        mode:=hardware \
        model:=med14_robotiq_2f
    ```

Together these bring up the robot description with the gripper attached, MoveIt, and the controllers for arm and gripper.

> [!IMPORTANT]
> Both machines must share `ROS_DOMAIN_ID` and use the same RMW implementation, and the `model` argument must match on both sides — MoveIt on the perception PC plans against the description published by the robot PC.

> [!TIP]
> Check the gripper's serial device with `ls /dev/ttyUSB*` before launching; `com_port` defaults to `/dev/ttyUSB1` and the device number can change between reboots.

## Motion API (`lbr_motion` + `lbr_interfaces`)

### Actions

Defined in [lbr_interfaces/action](lbr_interfaces/action/):

```
# MoveToPose.action                      # MoveHome.action
geometry_msgs/PoseStamped desired_pose   float32 vel_scaling
float32 vel_scaling                      float32 acc_scaling
float32 acc_scaling                      string  planning_group
string  planning_group                   ---
---                                      geometry_msgs/PoseStamped reached_pose
geometry_msgs/PoseStamped reached_pose   bool    success
bool    success                          string  message
string  message                          ---
---                                      geometry_msgs/PoseStamped current_pose
geometry_msgs/PoseStamped current_pose   string  state
string  state
```

`vel_scaling` and `acc_scaling` are clamped to `[0, 1]` by the servers and fall back to `0.1`. `planning_group` falls back to `arm`.

### Nodes

| Node (`ros2 run lbr_motion <exe>`) | Type | Description |
| :--- | :--- | :--- |
| `move_to_pose_server` | Action server `move_to_pose` (`lbr_interfaces/action/MoveToPose`) | Wraps MoveIt. Calls `/lbr/compute_ik` for the requested `PoseStamped`, builds a joint-space goal from the IK solution, and forwards it to MoveIt's `/lbr/move_action`. Rejects goals for which IK fails. |
| `move_home_server` | Action server `move_to_home` (`lbr_interfaces/action/MoveHome`) | Same wrapper without IK: commands every joint of the planning group to `0.0` and sends it to `/lbr/move_action`. |
| `move_to_pose_client` | Action client | Example client; edit the goal pose in `main()` (ships with position `(0, 0, 1.3)`, identity orientation, group `arm`, scalings `0.1`). Poses are specified as quaternions. |
| `move_home_client` | Action client | Sends a `MoveHome` goal for group `arm` and exits when the result arrives. |
| `joint_servo_pub` | Publisher | Publishes `control_msgs/JointJog` on `/lbr/servo_node/delta_joint_cmds` at 4 Hz to jog individual joints (`lbr_A2`, `lbr_A4`, `lbr_A6` in the shipped example). |
| `twist_servo_pub` | Publisher | Publishes `geometry_msgs/TwistStamped` on `/lbr/servo_node/delta_twist_cmds` at 100 Hz in the `lbr_link_ee` frame to jog the end effector. |
| `tf_tree_sub` | TF listener | Standalone node that looks up the base → end-effector transform at 100 Hz; set `_sub_echo=True` to log the pose. |

Both servers take a `robot_name` parameter (default `med14_tc`) that selects joint names, planning groups, and TF frames — the launch files pass the `model` argument through to it. Valid values are `med14`, `med14_tc`, and `med14_robotiq_2f`; anything else falls back to a bare 7-DoF arm configuration and logs a warning.

### Server behavior details

- **Topics/services consumed:** `lbr/joint_states` (joint state, 100-deep queue), `/lbr/compute_ik` (MoveIt IK service), `/lbr/move_action` (MoveIt `MoveGroup` action), TF.
- **Planning:** OMPL pipeline, 10 planning attempts, 5 s allowed planning time, empty path/trajectory constraints.
- **Workspace bounds:** planning is restricted to a ±1 m box around the origin of the `world` frame (`ws_bounds` in the server) — enlarge it there if your targets fall outside.
- **Concurrency:** both servers use a `ReentrantCallbackGroup` and a lock, so one goal executes at a time; new goals are rejected while a goal is active, and cancellation is forwarded to `move_action`.
- **Feedback:** the current end-effector pose (from TF, updated at 500 Hz) plus a planning `state` string.

Send a goal without writing a client:

```shell
ros2 action send_goal /move_to_pose lbr_interfaces/action/MoveToPose \
"{desired_pose: {header: {frame_id: world}, pose: {position: {x: 0.0, y: 0.0, z: 1.3}, orientation: {w: 1.0}}}, vel_scaling: 0.1, acc_scaling: 0.1, planning_group: arm}"

ros2 action send_goal /move_to_home lbr_interfaces/action/MoveHome \
"{vel_scaling: 0.1, acc_scaling: 0.1, planning_group: arm}"
```

## Gepetto hand bringup

`med14_gepetto` is the join between this stack and the [`gepetto_ros`](../gepetto_ros) /
[`gepetto_core`](../gepetto_core) tendon-driven hand. It adds one frame,
`lbr_gepetto_wrist_link`, fixed to `lbr_link_ee` at the measured mount offset:

```xml
<origin xyz="-0.009490 -0.010641 0.134688" rpy="1.570796 0.174533 -1.570796" />
```

Those numbers are `MountConfig.flange_from_wrist_xyz` / `_rpy` from
[gepetto_core/src/gepetto_core/config.py](../gepetto_core/src/gepetto_core/config.py), copied
verbatim — `gepetto_core`'s `transform_from_xyz_rpy` builds `R = Rz(yaw) @ Ry(pitch) @ Rx(roll)`,
which is exactly URDF's `rpy` convention, so no conversion is involved. The wrist link carries
marker geometry only; the hand itself is not modelled.

> [!IMPORTANT]
> Nothing detects a stale mount value. After any change to the Onshape assembly, the mounting
> bracket, or the hand morphology, re-run `python -m python.tests.tendon_hand.mount_onshape_fit`
> from `crest-sparse/` and update **both** `config.py` and
> [gepetto_hand_description.xacro](lbr_description/urdf/med14_gepetto/gepetto_hand_description.xacro).
> Verify with `ros2 run tf2_ros tf2_echo lbr_link_ee lbr_gepetto_wrist_link` against
> `HandConfig().mount.T_flange_from_wrist()` — the rotation is nearly a pair of right angles, so
> a flipped one still looks plausible.

Description + RViz + the finger slider panel:

```shell
ros2 launch lbr_bringup lbr_gepetto.launch.py
```

The same thing under resolved-rate control (MoveIt Servo on `forward_position_controller`):

```shell
ros2 launch lbr_bringup lbr_gepetto.launch.py servo:=true
# then, in a second terminal
ros2 run lbr_motion twist_servo_pub
```

| Argument | Default | Meaning |
| :--- | :--- | :--- |
| `servo` | `false` | `true` swaps `joint_trajectory_controller` for `forward_position_controller` and starts MoveIt Servo. The MoveIt include is condition-gated, so `servo:=false` does not need MoveIt installed. |
| `gepetto_nodes` | `sliders` | `sliders` = `finger_slider_node` only; `full` = `hand_node` + `state_estimator` + `planner`; `none` = arm only. |
| `conda_prefix` | `~/miniconda3/envs/crest_py10` | Env supplying `gepetto_core` and `crest_sparse` to the hand nodes. |
| `moving_speed`, `torque_limit`, `command_hz` | `0`, `0`, `20.0` | Passed to `finger_slider_node`; `0` keeps the `HandConfig` default. |

`gepetto_nodes` is one enum rather than separate switches on purpose: `finger_slider_node` and
`hand_node` both claim `/dev/ttyUSB*` and cannot run at the same time.

The conda environment is applied per-node via `additional_env` rather than a launch-wide
`SetEnvironmentVariable` (which is what the launch files in `gepetto_launch` do). That keeps
conda's `site-packages` out of `move_group`, `servo_node`, `ros2_control_node`, and RViz, which
must keep the system python they were built against.

## Servoing

Servoing jogs the robot continuously instead of planning point-to-point motions. It requires the **`forward_position_controller`**, which is mutually exclusive with the `joint_trajectory_controller` used by `move_to_pose` / MoveIt planning — so the servoing and move-to-pose stacks cannot run at the same time.

As long as something is publishing on the servo topics, the robot jogs; when publishing stops, it stops.

**Simulation:**

```shell
# terminal 1 - mock robot on forward_position_controller + MoveIt Servo + RViz
ros2 launch lbr_bringup lbr_servoing.launch.py
# ...or, to also spawn the Robotiq controllers in mock mode:
ros2 launch lbr_bringup lbr_servoing_and_robotiq.launch.py

# terminal 2 - pick one
ros2 run lbr_motion twist_servo_pub
ros2 run lbr_motion joint_servo_pub
```

**Hardware:**

```shell
# robot PC
ros2 launch lbr_bringup lbr_and_robotiq_hardware.launch.py ctrl:='forward_position_controller'

# perception PC
ros2 launch lbr_bringup lbr_servoing.launch.py mode:=hardware

# perception PC, pick one
ros2 run lbr_motion twist_servo_pub
ros2 run lbr_motion joint_servo_pub
```

Servo parameters live in [lbr_bringup/config/moveit_servo.yaml](lbr_bringup/config/moveit_servo.yaml).

> [!TIP]
> `twist_servo_pub` does not work when the robot is near a singularity (MoveIt Servo halts on the singularity threshold). Use `joint_servo_pub` to jog the joints away from the singularity, then switch back to twist servoing.

> [!NOTE]
> Both publishers ship with hardcoded example commands. Copy them and edit the twist/jog values, joint names, and frame IDs for your own use — `twist_servo_pub` commands in `lbr_link_ee`, `joint_servo_pub` stamps its header with `base_link`.

## Gripper control

For `med14_robotiq_2f`, [combined_controllers.yaml](lbr_description/ros2_control/combined_controllers.yaml) adds two controllers alongside the LBR ones:

| Controller | Type | Purpose |
| :--- | :--- | :--- |
| `robotiq_gripper_controller` | `position_controllers/GripperActionController` | Commands `lbr_finger_joint` (effort + speed interfaces enabled, `max_velocity: 0.1`). |
| `robotiq_activation_controller` | `robotiq_controllers/RobotiqActivationController` | Activates/re-activates the gripper hardware. |

They are spawned automatically by `lbr_and_robotiq_hardware.launch.py` (hardware) and by `lbr_move_to_pose.launch.py` / `lbr_servoing_and_robotiq.launch.py` (mock/gazebo).

Command the gripper directly through the controller's `GripperCommand` action (confirm the exact name with `ros2 action list`):

```shell
ros2 action send_goal /lbr/robotiq_gripper_controller/gripper_cmd control_msgs/action/GripperCommand \
"{command: {position: 0.7, max_effort: 50.0}}"   # 0.0 = open, ~0.7 = closed
```

MoveIt also knows the gripper: the `gripper` planning group in [med14_robotiq_2f.srdf](lbr_moveit_config/med14_robotiq_2f_moveit_config/config/med14_robotiq_2f.srdf) defines `gripper_open` (0.0) and `gripper_closed` (0.7) states, and `moveit_controllers.yaml` registers `robotiq_gripper_controller` as a `GripperCommand` controller — so you can also drive it from the RViz MotionPlanning panel or by passing `planning_group: gripper` to the motion servers. The mimic joints of the 2F-140 are declared passive so MoveIt does not try to plan for them.

## 3D sensors / octomap collision avoidance

`move_group.launch.py` (and therefore `lbr_move_to_pose.launch.py`) accepts:

| Argument | Default | Meaning |
| :--- | :--- | :--- |
| `sensors_3d` | `false` | Enable depth-sensor / octomap integration in `move_group`. |
| `sensors_3d_config` | `''` | **Absolute path** to a MoveIt `sensors_3d.yaml`. |

When both are set, the config is passed to `MoveItConfigsBuilder.sensors_3d()`, the octomap is configured (`octomap_resolution: 0.025`, `octomap_frame: world`, `max_range: 5.0`) and the `ClearOctomapService` capability is enabled:

```shell
ros2 launch lbr_bringup lbr_move_to_pose.launch.py \
    model:=med14_robotiq_2f \
    sensors_3d:=true \
    sensors_3d_config:=/absolute/path/to/sensors_3d.yaml
```

No sensor config ships with this repo — supply your own, e.g.:

```yaml
sensors:
  - default_sensor
default_sensor:
  sensor_plugin: occupancy_map_monitor/PointCloudOctomapUpdater
  point_cloud_topic: /camera/depth/color/points
  max_range: 5.0
  point_subsample: 1
  padding_offset: 0.1
  padding_scale: 1.0
  max_update_rate: 1.0
  filtered_cloud_topic: /filtered_cloud
```

The camera's frames must be connected to `world` in TF (via a static transform or a calibration publisher) or the octomap will stay empty.

## Which controller do I need?

| Task | Controller (`ctrl` argument) | Bring up with |
| :--- | :--- | :--- |
| MoveIt planning, `move_to_pose`, `move_to_home` | `joint_trajectory_controller` (default) | `lbr_move_to_pose.launch.py` (+ `lbr_and_robotiq_hardware.launch.py` on hardware) |
| Joint or twist servoing | `forward_position_controller` | `lbr_servoing.launch.py` (+ `lbr_and_robotiq_hardware.launch.py ctrl:=forward_position_controller`) |
| Direct FRI command demos | `lbr_joint_position_command_controller`, `lbr_torque_command_controller`, `lbr_wrench_command_controller`, `twist_controller`, `admittance_controller` | see [lbr_demos](lbr_demos/) |

Switching controllers at runtime:

```shell
ros2 control list_controllers -c /lbr/controller_manager
ros2 control switch_controllers -c /lbr/controller_manager \
    --deactivate joint_trajectory_controller \
    --activate forward_position_controller
```

## Troubleshooting

| Symptom | Likely cause / fix |
| :--- | :--- |
| `move_to_pose_server` logs "IK service not available" forever | `move_group` is not running (or is in another namespace). MoveIt must be up before the servers can connect to `/lbr/compute_ik` and `/lbr/move_action`. |
| Goal rejected immediately | IK found no solution for the requested pose — check the frame (`world`), reachability, and that `planning_group` matches the model. |
| Robot does not move although planning succeeds | Wrong controller active; MoveIt needs `joint_trajectory_controller`, servoing needs `forward_position_controller`. |
| Twist servoing stalls or halts | Near-singular configuration — jog away with `joint_servo_pub`. |
| xacro error about `robotiq_description` | `ros2_robotiq_gripper` is not in the workspace; see [Installation](#installation) step 3. |
| Gripper controller fails to start on hardware | Wrong `com_port`, or the user lacks permission on the serial device (`sudo usermod -aG dialout $USER`). |
| Joint states/topics missing on the perception PC | `ROS_DOMAIN_ID` mismatch, or the robot PC bringup is not running. |
| Wrong number of state interfaces / FRI mismatch | `fri_client_sdk` version in `lbr_system_config.yaml` does not match the FRI client you built against. |

## Documentation
Full upstream documentation is available on [Read the Docs](https://lbr-stack.readthedocs.io/en/latest). Fork-specific behavior is documented here.

## Citation
If you enjoyed using this repository for your work, we would really appreciate ❤️ if you could leave a ⭐ and / or cite it, as it helps us to continue offering support.

```
@article{Huber2024,
  doi       = {10.21105/joss.06138},
  url       = {https://doi.org/10.21105/joss.06138},
  year      = {2024},
  publisher = {The Open Journal},
  volume    = {9},
  number    = {103},
  pages     = {6138},
  author    = {Martin Huber and Christopher E. Mower and Sebastien Ourselin and Tom Vercauteren and Christos Bergeles},
  title     = {LBR-Stack: ROS 2 and Python Integration of KUKA FRI for Med and IIWA Robots},
  journal   = {Journal of Open Source Software}
}
```

## Acknowledgements
### Open Source Contributors
We would like to acknowledge all contributors 🚀

**lbr_fri_ros2_stack**

[![lbr_fri_ros2_stack contributors](https://contrib.rocks/image?repo=lbr-stack/lbr_fri_ros2_stack&max=20)](https://github.com/lbr-stack/lbr_fri_ros2_stack/graphs/contributors)

**fri**

[![fri contributors](https://contrib.rocks/image?repo=lbr-stack/fri&max=20)](https://github.com/lbr-stack/fri/graphs/contributors)

### Organizations and Grants
We would further like to acknowledge following supporters:

| Logo | Notes |
|:--:|:---|
| <img src="https://medicalengineering.org.uk/wp-content/themes/aalto-child/_assets/images/medicalengineering-logo.svg" alt="wellcome" width="150" align="left">  | This work was supported by core and project funding from the Wellcome/EPSRC [WT203148/Z/16/Z; NS/A000049/1; WT101957; NS/A000027/1]. |
| <img src="https://upload.wikimedia.org/wikipedia/commons/thumb/b/b7/Flag_of_Europe.svg/1920px-Flag_of_Europe.svg.png" alt="eu_flag" width="150" align="left"> | This project has received funding from the European Union's Horizon 2020 research and innovation programme under grant agreement No 101016985 (FAROS project). |
| <img src="https://rvim.online/author/avatar_hu8970a6942005977dc117387facf47a75_62303_270x270_fill_lanczos_center_2.png" alt="RViMLab" width="150" align="left"> | Built at [RViMLab](https://rvim.online/). |
| <img src="https://avatars.githubusercontent.com/u/75276868?s=200&v=4" alt="King's College London" width="150" align="left"> | Built at [CAI4CAI](https://cai4cai.ml/). |
| <img src="https://upload.wikimedia.org/wikipedia/commons/1/14/King%27s_College_London_logo.svg" alt="King's College London" width="150" align="left"> | Built at [King's College London](https://www.kcl.ac.uk/). |
