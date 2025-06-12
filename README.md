# lbr_fri_ros2_stack
[![License](https://img.shields.io/github/license/lbr-stack/lbr_fri_ros2_stack)](https://github.com/lbr-stack/lbr_fri_ros2_stack/tree/humble?tab=Apache-2.0-1-ov-file#readme) 
[![Documentation Status](https://readthedocs.org/projects/lbr-stack/badge/?version=latest)](https://lbr-stack.readthedocs.io/en/latest/?badge=latest)
[![JOSS](https://joss.theoj.org/papers/c43c82bed833c02503dd47f2637192ef/status.svg)](https://joss.theoj.org/papers/c43c82bed833c02503dd47f2637192ef) 
[![Code Style: Black](https://img.shields.io/badge/code%20style-black-000000.svg)](https://github.com/psf/black)

ROS 2 packages for the KUKA LBR, including communication to the real robot via the Fast Robot Interface ([FRI](https://github.com/lbr-stack/fri)) and [Gazebo](http://gazebosim.org/) simulation support. Included are the `iiwa7`, `iiwa14`, `med7`, and `med14`.

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

## Status
| OS             | ROS Distribution | FRI Version |  Build Status |
| :------------- | :--------------- | :---------- |  :----------- |
| `Ubuntu-22.04` | `humble`         | `1.11`      |  [![build-ubuntu-22.04-fri-1.11](https://github.com/lbr-stack/lbr_fri_ros2_stack/actions/workflows/build-ubuntu-22.04-fri-1.11.yml/badge.svg)](https://github.com/lbr-stack/lbr_fri_ros2_stack/actions/workflows/build-ubuntu-22.04-fri-1.11.yml) |
| `Ubuntu-22.04` | `humble`         | `1.14`      |  [![build-ubuntu-22.04-fri-1.14](https://github.com/lbr-stack/lbr_fri_ros2_stack/actions/workflows/build-ubuntu-22.04-fri-1.14.yml/badge.svg)](https://github.com/lbr-stack/lbr_fri_ros2_stack/actions/workflows/build-ubuntu-22.04-fri-1.14.yml) |
| `Ubuntu-22.04` | `humble`         | `1.15`      |  [![build-ubuntu-22.04-fri-1.15](https://github.com/lbr-stack/lbr_fri_ros2_stack/actions/workflows/build-ubuntu-22.04-fri-1.15.yml/badge.svg)](https://github.com/lbr-stack/lbr_fri_ros2_stack/actions/workflows/build-ubuntu-22.04-fri-1.15.yml) |
| `Ubuntu-22.04` | `humble`         | `1.16`      |  [![build-ubuntu-22.04-fri-1.16](https://github.com/lbr-stack/lbr_fri_ros2_stack/actions/workflows/build-ubuntu-22.04-fri-1.16.yml/badge.svg)](https://github.com/lbr-stack/lbr_fri_ros2_stack/actions/workflows/build-ubuntu-22.04-fri-1.16.yml) |
| `Ubuntu-22.04` | `humble`         | `2.5`      |  [![build-ubuntu-22.04-fri-2.5](https://github.com/lbr-stack/lbr_fri_ros2_stack/actions/workflows/build-ubuntu-22.04-fri-2.5.yml/badge.svg)](https://github.com/lbr-stack/lbr_fri_ros2_stack/actions/workflows/build-ubuntu-22.04-fri-2.5.yml) |
| `Ubuntu-22.04` | `humble`         | `2.7`      |  [![build-ubuntu-22.04-fri-2.7](https://github.com/lbr-stack/lbr_fri_ros2_stack/actions/workflows/build-ubuntu-22.04-fri-2.7.yml/badge.svg)](https://github.com/lbr-stack/lbr_fri_ros2_stack/actions/workflows/build-ubuntu-22.04-fri-2.7.yml) |

## Documentation
Full documentation available on [Read the Docs](https://lbr-stack.readthedocs.io/en/latest).

## Quick Start
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

> [!NOTE]
> FRI client is cloned from [fri](https://github.com/lbr-stack/fri) and must be available as branch, refer [README](https://github.com/lbr-stack/fri?tab=readme-ov-file#contributing).

3. Build

    ```shell
    colcon build --symlink-install
    ```

4. In terminal 1, launch a mock setup via

    ```shell
    source install/setup.bash
    ros2 launch lbr_bringup mock.launch.py \
        model:=iiwa7 # [iiwa7, iiwa14, med7, med14]
    ```

> [!TIP]
> List all arguments for the launch file via `ros2 launch lbr_bringup mock.launch.py -s`

5. In terminal 2, visualize the setup via

    ```shell
    source install/setup.bash
    ros2 launch lbr_bringup rviz.launch.py \
        rviz_cfg_pkg:=lbr_bringup \
        rviz_cfg:=config/mock.rviz
    ```

Now, run the [demos](https://lbr-stack.readthedocs.io/en/latest/lbr_fri_ros2_stack/lbr_demos/doc/lbr_demos.html). To get started with the real robot, checkout the [Hardware Setup](https://lbr-stack.readthedocs.io/en/latest/lbr_fri_ros2_stack/lbr_fri_ros2_stack/doc/hardware_setup.html).


6. The custom move_to_pose nodes can be launched in simulation mode via

    ```shell
    ros2 launch lbr_bringup lbr_move_to_pose.launch.py model:=med14_robotiq_2f
    ```

7. The move_to_pose nodes can be launched in hardware mode via 

    ```shell
    ros2 launch lbr_bringup lbr_move_to_pose.launch.py mode:=hardware model:=med14_robotiq_2f
    ```
    on the Peception PC (which doesn't have a realtime kernel) and 
    ```shell
    ros2 launch lbr_bringup lbr_and_robotiq_hardware.launch.py
    ```
    on the robot PC (which is connected to the KUKA controller and the Robotiq controller and is running a realtime/low-latency kernel).

    These launch files launch the robot description with the gripper attached to the end of the robot, the MoveIt services, and the controllers for the robot and gripper (either in sim or on hardware depending on which mode you chose).

8. Custom nodes can be found in the lbr_motion package. These nodes include:
    - joint_servo_pub: which publishes joint servoing commands to "jog" the robot joints
    - move_home_client: which publishes a request to send the robot to the home position
    - move_home_server: which is a wrapper aroung MoveIt that commands the robot to move to the home position (all joint angles are 0)
    - move_to_pose_client: which is a server that sends a request to move the robot to a specific end-effector pose
    - move_top_pose_server: which is a wrapper around MoveIt that takes the end-effector pose request, finds an IK solution, and then senda a planning request to MoveIt to command the robot to that IK solution
    - pose_pub: which publishes poses
    - tf_tree_sub: which subscribes to the tf tree
    - twist_servo_pub: which publishes a twist servo command to the robot, requesting that the end-effector pose be jogged by that amount. This is done in a collision free way using MoveIt.

> [!TIP]
> The twist_servo_pub does not work when the robot is close to a singularity so using the joint_servo_pub to get the robot away from a singularity can fix this issue.

9. To do joint servoing or end-effector twist servoing the robot needs to be using the forward position controller. This requires a different setup from running the move_to_pose server which requires the robot to be using the position controller. As long as there is a topic being published on either the joint servo or twist servo topics, the robot will jog. Once the topic stops publishing the robot stops moving. Launch twist servoing in simulation via:
    
    ```shell
    ros2 launch lbr_bringup lbr_servoing.launch.py
    ```

    ```shell
    ros2 run lbr_motion twist_servo_pub
    ```
    or
    ```shell
    ros2 run lbr_motion joint_servo_pub
    ```

    For hardware:

    ```shell
    ros2 launch lbr_bringup lbr_and_robotiq_hardware.launch.py ctrl:='forward_position_controller'
    ```
    
    ```shell
    ros2 launch lbr_bringup lbr_servoing.launch.py mode:=hardware
    ```

    ```shell
    ros2 run lbr_motion twist_servo_pub
    ```
    or
    ```shell
    ros2 run lbr_motion joint_servo_pub
    ```



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
