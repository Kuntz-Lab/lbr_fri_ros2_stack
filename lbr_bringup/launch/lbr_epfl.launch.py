"""Combined bringup for the med14 + EPFL tendon-driven hand.

Brings up the `med14_epfl` description (the arm, its table, and the hand's wrist
frame at the fixed flange->wrist mount offset), RViz, and the EPFL hand nodes from
`epfl_hand_ros` in one shot.

Two modes of arm control, selected by `servo`:

    servo:=true   (default)  forward_position_controller + MoveIt Servo, i.e. resolved
                             rate control: Servo maps an incoming Cartesian twist to
                             joint velocities through the Jacobian pseudo-inverse and
                             publishes to /lbr/forward_position_controller/commands.
                             This is how the cell is actually driven --
                             gepetto_ros's executor_node and home_node, and
                             lbr_motion's *_servo_pub, all command Servo.
    servo:=false             joint_trajectory_controller -- plain description bringup,
                             for eyeballing the mount transform in RViz. Needs no MoveIt.

The MoveIt Servo include is condition-gated, so with servo:=false nothing in this
launch imports MoveIt.

THE VISER WORKBENCH IS NOT HERE. It used to be, as `epfl_nodes:=viz`, which also
started the trajectory executor and the finger servo. Both the workbench and the
executor are hand-agnostic and now live in `gepetto_ros`, which INCLUDES this
launch with `epfl_nodes:=none` and adds them itself:

    ros2 launch gepetto_ros_launch gepetto_bringup.launch.py hand:=epfl mode:=mock

That is the entry point for solving in a browser and playing it on the robot. This
stack stays what it is -- descriptions, ros2_control, MoveIt, servo and RViz -- and
carries no dependency on a solver or a visualizer.

Examples:

    # the usual thing from this stack: resolved rate control, RViz, and the arm
    # ready to be driven by whatever commands Servo (gepetto_ros's executor,
    # lbr_motion's twist_servo_pub, or your own node)
    ros2 launch lbr_bringup lbr_epfl.launch.py epfl_nodes:=none

    # ...paired with a hand you start yourself:
    #   ros2 run epfl_hand_hardware finger_servo_node --ros-args -p dry_run:=true

    # open-loop finger sliders, for bring-up with no planner in the loop
    ros2 launch lbr_bringup lbr_epfl.launch.py epfl_nodes:=sliders

    # description + RViz only, no MoveIt
    ros2 launch lbr_bringup lbr_epfl.launch.py servo:=false epfl_nodes:=none
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    EnvironmentVariable,
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    ld = LaunchDescription()

    # -- arguments ----------------------------------------------------------
    ld.add_action(
        DeclareLaunchArgument(
            "model",
            default_value="med14_epfl",
            description="Robot model to use.",
        )
    )
    model = LaunchConfiguration("model")

    ld.add_action(
        DeclareLaunchArgument(
            "mode",
            default_value="mock",
            description="Robot operation mode.",
            choices=["mock", "hardware", "gazebo"],
        )
    )
    mode = LaunchConfiguration("mode")

    ld.add_action(
        DeclareLaunchArgument(
            "servo",
            default_value="true",
            description="Run under MoveIt Servo (resolved rate control) on "
            "forward_position_controller instead of joint_trajectory_controller.",
            choices=["true", "false"],
        )
    )
    servo = LaunchConfiguration("servo")

    ld.add_action(
        DeclareLaunchArgument(
            "rviz",
            default_value="true",
            description="Whether to start RViz.",
        )
    )
    rviz = LaunchConfiguration("rviz")

    ld.add_action(
        DeclareLaunchArgument(
            "epfl_nodes",
            default_value="none",
            description="Which hand nodes to start. 'sliders' is the open-loop slider "
            "GUI, 'full' is hand_node + state_estimator + planner, 'none' starts "
            "nothing and is the default -- pair it with a hand node you start "
            "yourself, or use gepetto_ros_launch's gepetto_bringup.launch.py, "
            "which includes this launch and adds the executor, the finger servo "
            "and the viser workbench. These are mutually exclusive: "
            "finger_slider_node, hand_node and finger_servo_node all claim "
            "/dev/ttyUSB*.",
            choices=["sliders", "full", "none"],
        )
    )
    epfl_nodes = LaunchConfiguration("epfl_nodes")

    ld.add_action(
        DeclareLaunchArgument(
            "moving_speed",
            default_value="0",
            description="Dynamixel moving speed 1-1023 for the sliders; 0 keeps the "
            "HandConfig default. ~200 is slow enough to watch.",
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            "torque_limit",
            default_value="0",
            description="Dynamixel torque limit for the sliders; 0 keeps the "
            "HandConfig default (300).",
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            "command_hz",
            default_value="20.0",
            description="How often the slider GUI writes changed values to the motors.",
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            "conda_prefix",
            default_value=[EnvironmentVariable("HOME"), "/miniconda3/envs/gepetto_py10"],
            description="Conda env the epfl_hand_ros nodes run against. It must "
            "provide epfl_hand_core, which is what the hardware nodes import. "
            "NOTE that epfl_nodes:=full ALSO starts state_estimator and planner, "
            "which import crest_sparse and therefore want the deprecated "
            "crest_py10 env instead -- those two have not been ported and that "
            "combination is unverified. See epfl_hand_ros/README.md.",
        )
    )
    conda_prefix = LaunchConfiguration("conda_prefix")

    lbr_bringup_path = FindPackageShare("lbr_bringup")

    # -- arm ----------------------------------------------------------------
    # Servoing needs forward_position_controller; it is mutually exclusive with the
    # joint_trajectory_controller MoveIt planning uses.
    ctrl = PythonExpression(
        [
            "'forward_position_controller' if '",
            servo,
            "' == 'true' else 'joint_trajectory_controller'",
        ]
    )

    # In hardware mode the description and controllers come from the robot PC.
    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [PathJoinSubstitution([lbr_bringup_path, "launch", "mock.launch.py"])]
            ),
            launch_arguments={
                "model": model,
                "ctrl": ctrl,
            }.items(),
            condition=IfCondition(
                PythonExpression(
                    ["'", mode, "' == 'mock' or '", mode, "' == 'gazebo'"]
                )
            ),
        )
    )

    # -- resolved rate control ----------------------------------------------
    # Gated, so servo:=false never imports MoveIt.
    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [
                    PathJoinSubstitution(
                        [lbr_bringup_path, "launch", "moveit_servo.launch.py"]
                    )
                ]
            ),
            launch_arguments={
                "model": model,
                "mode": mode,
            }.items(),
            condition=IfCondition(servo),
        )
    )

    # -- rviz ---------------------------------------------------------------
    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [PathJoinSubstitution([lbr_bringup_path, "launch", "rviz.launch.py"])]
            ),
            condition=IfCondition(rviz),
        )
    )

    # -- EPFL hand nodes from epfl_hand_ros -----------------------------------
    # The epfl_hand_ros nodes need the conda env that carries epfl_hand_core and
    # crest_sparse, while every other node in this launch must keep the system python
    # that rclpy and MoveIt were built against. So the env goes on the individual nodes
    # via additional_env rather than through a launch-wide SetEnvironmentVariable,
    # which would leak conda's site-packages into move_group, servo_node and RViz.
    epfl_hand_ros_env = {
        "PYTHONPATH": [
            conda_prefix,
            "/lib/python3.10/site-packages:",
            EnvironmentVariable("PYTHONPATH", default_value=""),
        ],
        "LD_LIBRARY_PATH": [
            conda_prefix,
            "/lib:",
            EnvironmentVariable("LD_LIBRARY_PATH", default_value=""),
        ],
    }

    use_sliders = IfCondition(
        PythonExpression(["'", epfl_nodes, "' == 'sliders'"])
    )
    use_full = IfCondition(PythonExpression(["'", epfl_nodes, "' == 'full'"]))

    ld.add_action(
        Node(
            package="epfl_hand_hardware",
            executable="finger_slider_node",
            name="finger_slider_node",
            output="screen",
            additional_env=epfl_hand_ros_env,
            # Substitutions resolve to strings, so each one is typed explicitly;
            # otherwise the node rejects them against its declared parameter types.
            parameters=[
                {
                    "command_hz": ParameterValue(
                        LaunchConfiguration("command_hz"), value_type=float
                    ),
                    "moving_speed": ParameterValue(
                        LaunchConfiguration("moving_speed"), value_type=int
                    ),
                    "torque_limit": ParameterValue(
                        LaunchConfiguration("torque_limit"), value_type=int
                    ),
                }
            ],
            condition=use_sliders,
        )
    )

    ld.add_action(
        Node(
            package="epfl_hand_hardware",
            executable="hand_node",
            name="hand_node",
            output="screen",
            additional_env=epfl_hand_ros_env,
            condition=use_full,
        )
    )
    ld.add_action(
        Node(
            package="epfl_hand_control",
            executable="state_estimator",
            name="state_estimator",
            output="screen",
            additional_env=epfl_hand_ros_env,
            condition=use_full,
        )
    )
    ld.add_action(
        Node(
            package="epfl_hand_control",
            executable="planner",
            name="planner",
            output="screen",
            additional_env=epfl_hand_ros_env,
            condition=use_full,
        )
    )

    return ld
