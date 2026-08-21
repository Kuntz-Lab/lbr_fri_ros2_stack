"""Combined bringup for the med14 + Gepetto tendon-driven hand.

Brings up the `med14_gepetto` description (the arm, its table, and the hand's wrist
frame at the fixed flange->wrist mount offset), RViz, and the Gepetto hand nodes in
one shot.

Two modes of arm control, selected by `servo`:

    servo:=false  (default)  joint_trajectory_controller -- plain description bringup,
                             for eyeballing the mount transform in RViz. Needs no MoveIt.
    servo:=true              forward_position_controller + MoveIt Servo, i.e. resolved
                             rate control: Servo maps an incoming Cartesian twist to
                             joint velocities through the Jacobian pseudo-inverse and
                             publishes to /lbr/forward_position_controller/commands.

The MoveIt Servo include is condition-gated, so with servo:=false nothing in this
launch imports MoveIt.

Examples:

    # description + RViz + finger sliders
    ros2 launch lbr_bringup lbr_gepetto.launch.py

    # resolved rate control + finger sliders, then drive it with
    #   ros2 run lbr_motion twist_servo_pub
    ros2 launch lbr_bringup lbr_gepetto.launch.py servo:=true

    # arm only
    ros2 launch lbr_bringup lbr_gepetto.launch.py gepetto_nodes:=none

    # resolved rate control + the interactive visualizer wired to the robot,
    # at http://localhost:8080 -- solve in the browser, play it on the hardware
    ros2 launch lbr_bringup lbr_gepetto.launch.py servo:=true gepetto_nodes:=viz
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
            default_value="med14_gepetto",
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
            default_value="false",
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
            "gepetto_nodes",
            default_value="sliders",
            description="Which hand nodes to start. 'sliders' is the open-loop slider "
            "GUI, 'full' is hand_node + state_estimator + planner, 'viz' is the "
            "interactive viser visualizer in ROS mode + finger_servo_node (the one "
            "that can play a solve on the robot). These are mutually exclusive: "
            "finger_slider_node, hand_node and finger_servo_node all claim "
            "/dev/ttyUSB*.",
            choices=["sliders", "full", "viz", "none"],
        )
    )
    gepetto_nodes = LaunchConfiguration("gepetto_nodes")

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
            default_value=[EnvironmentVariable("HOME"), "/miniconda3/envs/crest_py10"],
            description="Conda env the Gepetto nodes run against. It must provide "
            "gepetto_core and crest_sparse.",
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

    # -- gepetto hand nodes -------------------------------------------------
    # The Gepetto nodes need the conda env that carries gepetto_core and crest_sparse,
    # while every other node in this launch must keep the system python that rclpy and
    # MoveIt were built against. So the env goes on the individual nodes via
    # additional_env rather than through a launch-wide SetEnvironmentVariable, which
    # would leak conda's site-packages into move_group, servo_node and RViz.
    gepetto_env = {
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
        PythonExpression(["'", gepetto_nodes, "' == 'sliders'"])
    )
    use_full = IfCondition(PythonExpression(["'", gepetto_nodes, "' == 'full'"]))
    use_viz = IfCondition(PythonExpression(["'", gepetto_nodes, "' == 'viz'"]))

    ld.add_action(
        Node(
            package="gepetto_hardware",
            executable="finger_slider_node",
            name="finger_slider_node",
            output="screen",
            additional_env=gepetto_env,
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
            package="gepetto_hardware",
            executable="hand_node",
            name="hand_node",
            output="screen",
            additional_env=gepetto_env,
            condition=use_full,
        )
    )
    ld.add_action(
        Node(
            package="gepetto_control",
            executable="state_estimator",
            name="state_estimator",
            output="screen",
            additional_env=gepetto_env,
            condition=use_full,
        )
    )
    ld.add_action(
        Node(
            package="gepetto_control",
            executable="planner",
            name="planner",
            output="screen",
            additional_env=gepetto_env,
            condition=use_full,
        )
    )

    # gepetto_nodes:=viz -- the interactive visualizer, wired to this robot. Pair
    # it with servo:=true: the visualizer's "Play solve on robot" publishes
    # Cartesian twists to MoveIt Servo, which only runs in that mode. It talks to
    # the hand through finger_servo_node below rather than owning the bus itself.
    ld.add_action(
        Node(
            package="gepetto_control",
            executable="viz_node",
            name="gepetto_viz",
            output="screen",
            additional_env=gepetto_env,
            condition=use_viz,
        )
    )
    ld.add_action(
        Node(
            package="gepetto_hardware",
            executable="finger_servo_node",
            name="finger_servo_node",
            output="screen",
            additional_env=gepetto_env,
            parameters=[
                {
                    "moving_speed": ParameterValue(
                        LaunchConfiguration("moving_speed"), value_type=int
                    ),
                    "torque_limit": ParameterValue(
                        LaunchConfiguration("torque_limit"), value_type=int
                    ),
                }
            ],
            condition=use_viz,
        )
    )

    return ld
