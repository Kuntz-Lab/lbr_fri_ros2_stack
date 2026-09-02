"""Combined bringup for the med14 + Allegro Hand (right, geared/type B).

Brings up RViz, MoveIt Servo and the Allegro hand node against the `med14_allegro`
description (the arm, its table, the 35 mm flange mount and the full 16-DOF hand), so both
the arm and the hand can be streamed commands at the same time.

WHAT THIS LAUNCH DOES NOT DO: with mode:=hardware it does not start the arm. The
description, ros2_control and the arm controllers come from the robot PC, which runs

    ros2 launch lbr_bringup hardware.launch.py model:=med14_allegro \
        ctrl:=forward_position_controller

on its own -- hardware.launch.py opens the FRI connection and must live next to the
controller, so it is deliberately NOT included from here. Start it there first, then start
this launch. (mode:=mock and mode:=gazebo do include mock.launch.py, since there is no
robot PC in those cases.) This matches lbr_epfl.launch.py.

Use ctrl:=joint_trajectory_controller on the robot PC instead if you launch this with
servo:=false; the two controllers claim the same interfaces and are mutually exclusive.

Two independent command paths, both continuous:

    arm    MoveIt Servo, i.e. resolved rate control -- Servo maps an incoming Cartesian
           twist to joint velocities through the Jacobian pseudo-inverse and publishes to
           /lbr/forward_position_controller/commands.

               /lbr/servo_node/delta_twist_cmds   geometry_msgs/TwistStamped

    hand   the Allegro driver's native joint servoing. allegro_node_grasp reads the 16
           positions POSITIONALLY out of the message (it ignores the `name` field) and
           immediately enters eMotionType_JOINT_PD, so no priming lib_cmd is needed --
           just publish at rate.

               /allegroHand/joint_cmd             sensor_msgs/JointState

The two halves meet at /lbr/joint_states: the hand node's joint_states are remapped onto
it, alongside the arm's joint_state_broadcaster. robot_state_publisher emits TF only for
the joints named in each message it receives, so the two publishers coexist rather than
fight, and Servo's CurrentStateMonitor takes the partial updates the same way. The result
is one robot_description, one TF tree, one planning scene containing the fingers.

`servo:=false` swaps forward_position_controller for joint_trajectory_controller and skips
the Servo include entirely (nothing then imports MoveIt) -- useful for eyeballing the
mount transform in RViz.

CAN is NOT configured by this launch. Bring the bus up first:

    sudo ip link set can0 down
    sudo ip link set can0 type can bitrate 1000000
    sudo ip link set can0 up

allegro_hand.launch.py does this inline, but it does so by blocking on an interactive
getpass() prompt, which would stall this whole bringup.

Examples:

    # everything faked, no hardware needed: arm on mock ros2_control, hand on the mock
    # node. Same command topics as the real thing.
    ros2 launch lbr_bringup lbr_allegro.launch.py

    # the real cell -- AFTER hardware.launch.py is already up on the robot PC
    ros2 launch lbr_bringup lbr_allegro.launch.py mode:=hardware

    # real arm, no hand plugged in
    ros2 launch lbr_bringup lbr_allegro.launch.py mode:=hardware hand:=none

    # description + RViz only, no MoveIt, for checking the 35 mm mount
    ros2 launch lbr_bringup lbr_allegro.launch.py servo:=false hand:=none
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    ld = LaunchDescription()

    # -- arguments ----------------------------------------------------------
    ld.add_action(
        DeclareLaunchArgument(
            "model",
            default_value="med14_allegro",
            description="Robot model to use.",
        )
    )
    model = LaunchConfiguration("model")

    ld.add_action(
        DeclareLaunchArgument(
            "mode",
            default_value="mock",
            description="Robot operation mode. Applies to the ARM only; the hand is "
            "selected separately with `hand`.",
            choices=["mock", "hardware", "gazebo"],
        )
    )
    mode = LaunchConfiguration("mode")

    ld.add_action(
        DeclareLaunchArgument(
            "servo",
            default_value="true",
            description="Run the arm under MoveIt Servo (resolved rate control) on "
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

    # Defaulted off `mode` rather than hardcoded, so the common cases -- all mock, or all
    # real -- need no extra argument, while `hand:=none` still covers a real arm with the
    # hand unplugged.
    ld.add_action(
        DeclareLaunchArgument(
            "hand",
            default_value=PythonExpression(
                ["'driver' if '", mode, "' == 'hardware' else 'mock'"]
            ),
            description="Which hand node to start. 'driver' is allegro_node_grasp on the "
            "CAN bus, 'mock' is allegro_mock_node.py (same two topics, no bus), 'none' "
            "starts nothing. Defaults to 'driver' when mode:=hardware, else 'mock'.",
            choices=["driver", "mock", "none"],
        )
    )
    hand = LaunchConfiguration("hand")

    ld.add_action(
        DeclareLaunchArgument(
            "which_hand",
            default_value="right",
            description="Which physical Allegro hand the driver talks to. Note the "
            "med14_allegro description models the RIGHT type-B hand; changing this "
            "without changing the description would mismatch the two.",
            choices=["right", "left"],
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            "which_type",
            default_value="B",
            description="Allegro hand type: A (non-geared) or B (geared).",
            choices=["A", "B"],
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            "can_device",
            default_value="can0",
            description="CAN interface the Allegro driver opens. Bring it up before "
            "launching; see this file's docstring.",
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            "polling",
            default_value="true",
            description="Whether the Allegro driver polls the CAN bus.",
        )
    )

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

    # In hardware mode the description and controllers come from the robot PC, which runs
    # hardware.launch.py itself; see this file's docstring. So only mock/gazebo bring up an
    # arm stack here.
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
                # The hand changes what proximity thresholds make sense, so this model
                # carries its own Servo config rather than the one every other model
                # shares. See the comments in that file for the two differences.
                "servo_cfg_pkg": "lbr_bringup",
                "servo_cfg": "config/moveit_servo_allegro.yaml",
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

    # -- allegro hand -------------------------------------------------------
    # joint_states is remapped into the arm's namespace so the combined
    # robot_state_publisher (which runs under /lbr) draws the fingers too. joint_cmd is
    # left global on purpose: it is the hand's servoing input and should not move around
    # depending on what the arm is called.
    #
    # The driver keeps its own ~333 Hz CAN loop and is not a ros2_control component. That
    # is deliberate -- it means no new hardware plugin, and the hand keeps working exactly
    # as it does under allegro_hand.launch.py.
    hand_remappings = [("allegroHand/joint_states", "/lbr/joint_states")]

    ld.add_action(
        Node(
            package="allegro_hand_controllers",
            executable="allegro_node_grasp",
            name="allegro_node_grasp",
            output="screen",
            parameters=[
                {"hand_info/which_hand": LaunchConfiguration("which_hand")},
                {"hand_info/which_type": LaunchConfiguration("which_type")},
                {"comm/CAN_CH": LaunchConfiguration("can_device")},
            ],
            arguments=[LaunchConfiguration("polling")],
            remappings=hand_remappings,
            condition=IfCondition(PythonExpression(["'", hand, "' == 'driver'"])),
        )
    )

    ld.add_action(
        Node(
            package="allegro_hand_controllers",
            executable="allegro_mock_node.py",
            name="allegro_mock_node",
            output="screen",
            remappings=hand_remappings,
            condition=IfCondition(PythonExpression(["'", hand, "' == 'mock'"])),
        )
    )

    return ld
