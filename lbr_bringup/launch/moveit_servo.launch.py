from typing import List

from launch import LaunchContext, LaunchDescription, LaunchDescriptionEntity
from launch.actions import DeclareLaunchArgument, OpaqueFunction, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessStart
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from lbr_bringup.description import LBRDescriptionMixin
from lbr_bringup.moveit import LBRMoveGroupMixin, LBRMoveItServoMixin


def hidden_setup(context: LaunchContext) -> List[LaunchDescriptionEntity]:
    ld = LaunchDescription()

    # Overridable so a model whose end effector changes what thresholds make sense can
    # supply its own; the defaults are the config every existing launch already used.
    moveit_servo_config = PathJoinSubstitution(
        [
            FindPackageShare(LaunchConfiguration("servo_cfg_pkg")),
            LaunchConfiguration("servo_cfg"),
        ]
    )
    model = LaunchConfiguration("model").perform(context)
    moveit_configs = LBRMoveGroupMixin.moveit_configs_builder(
        robot_name=model,
        package_name=f"{model}_moveit_config",
    ).to_moveit_configs()

    mode = LaunchConfiguration("mode").perform(context)
    use_sim_time = False
    if mode == "gazebo":
        use_sim_time = True

    # moveit servo node
    servo_node = LBRMoveItServoMixin.node_moveit_servo(
        parameters=[
            moveit_configs.robot_description_kinematics,
            moveit_configs.robot_description_semantic,
            {"use_sim_time": use_sim_time},
            moveit_servo_config,
            {
                "moveit_servo.use_gazebo": mode
                == "gazebo",  # we configure this parameter dynamically
            },
        ],
    )
    ld.add_action(servo_node)

    # call start servo after servo node start
    ld.add_action(
        RegisterEventHandler(
            OnProcessStart(
                target_action=servo_node,
                on_start=[
                    LBRMoveItServoMixin.call_start_servo_service(
                        condition=IfCondition(
                            LaunchConfiguration("default_enable_servo")
                        )
                    )
                ],
            ),
        )
    )

    return ld.entities


def generate_launch_description() -> LaunchDescription:
    ld = LaunchDescription()

    ld.add_action(LBRDescriptionMixin.arg_mode())
    ld.add_action(LBRDescriptionMixin.arg_model())
    ld.add_action(LBRMoveItServoMixin.arg_default_enable_servo())
    ld.add_action(
        DeclareLaunchArgument(
            name="servo_cfg_pkg",
            default_value="lbr_bringup",
            description="Package containing the MoveIt Servo configuration.",
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            name="servo_cfg",
            default_value="config/moveit_servo.yaml",
            description="Relative path from servo_cfg_pkg to the Servo configuration.",
        )
    )

    ld.add_action(OpaqueFunction(function=hidden_setup))
    return ld
