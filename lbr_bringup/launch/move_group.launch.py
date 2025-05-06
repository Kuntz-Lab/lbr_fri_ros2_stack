from typing import List

from launch import LaunchContext, LaunchDescription, LaunchDescriptionEntity
from launch.actions import OpaqueFunction, DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from lbr_bringup.description import LBRDescriptionMixin
from lbr_bringup.moveit import LBRMoveGroupMixin
from lbr_bringup.rviz import RVizMixin


def hidden_setup(context: LaunchContext) -> List[LaunchDescriptionEntity]:
    ld = LaunchDescription()

    ld.add_action(LBRDescriptionMixin.arg_robot_name())
    ld.add_action(LBRMoveGroupMixin.arg_allow_trajectory_execution())
    ld.add_action(LBRMoveGroupMixin.arg_capabilities())
    ld.add_action(LBRMoveGroupMixin.arg_disable_capabilities())
    ld.add_action(LBRMoveGroupMixin.arg_monitor_dynamics())
    ld.add_action(LBRMoveGroupMixin.args_publish_monitored_planning_scene())
    ld.add_action(LBRMoveGroupMixin.arg_sensors_3d())
    ld.add_action(LBRMoveGroupMixin.arg_sensors_3d_config())

    model = LaunchConfiguration("model").perform(context)

    # Check if 3D sensors should be used
    use_sensors_3d = LaunchConfiguration("sensors_3d").perform(context)
    config_file = LaunchConfiguration("sensors_3d_config").perform(context) if use_sensors_3d.lower() == 'true' else ""

    # Build the moveit configs based on whether sensors are enabled
    if use_sensors_3d.lower() == 'true' and config_file:
        moveit_configs_builder = (
            LBRMoveGroupMixin.moveit_configs_builder(
                robot_name=model,
                package_name=f"{model}_moveit_config",
            )
            .sensors_3d(file_path=config_file)
        )
    else:        
        moveit_configs_builder = LBRMoveGroupMixin.moveit_configs_builder(
            robot_name=model,
            package_name=f"{model}_moveit_config",
        )
    
    move_group_params = LBRMoveGroupMixin.params_move_group()

    mode = LaunchConfiguration("mode").perform(context)
    use_sim_time = False
    if mode == "gazebo":
        use_sim_time = True
    
    # Add sensor configuration if enabled
    parameters = [
        moveit_configs_builder.to_dict(),
        move_group_params,
        {"use_sim_time": use_sim_time},
    ]
    
    if use_sensors_3d.lower() == 'true' and config_file:
        # Add octomap parameters
        parameters.append({"octomap_resolution": 0.025})
        parameters.append({"octomap_frame": "world"})
        parameters.append({"use_depth_sensor": True})

        # Add sensor capability
        capabilities = "move_group/MoveGroupCartesianPathService move_group/MoveGroupKinematicsService move_group/MoveGroupExecuteTrajectoryAction move_group/MoveGroupMoveAction move_group/MoveGroupPickPlaceAction move_group/MoveGroupPlanService move_group/MoveGroupQueryPlannersService move_group/MoveGroupStateValidationService move_group/MoveGroupGetPlanningSceneService move_group/ApplyPlanningSceneService move_group/ClearOctomapService move_group/MoveGroupServoService"
        # Add sensor capability
        capabilities = capabilities + " move_group/MoveGroupSensorIntegrationService"
        parameters.append({"capabilities": capabilities})

    # MoveGroup
    robot_name = LaunchConfiguration("robot_name")
    ld.add_action(
        LBRMoveGroupMixin.node_move_group(
            parameters=parameters,
            namespace=robot_name,
        )
    )

    # # MoveGroup
    # robot_name = LaunchConfiguration("robot_name")
    # ld.add_action(
    #     LBRMoveGroupMixin.node_move_group(
    #         parameters=[
    #             moveit_configs_builder.to_dict(),
    #             move_group_params,
    #             {"use_sim_time": use_sim_time},
    #         ],
    #         namespace=robot_name,
    #     )
    # )

    # RViz if desired
    rviz = RVizMixin.node_rviz(
        rviz_cfg_pkg=f"{model}_moveit_config",
        rviz_cfg="config/moveit.rviz",
        parameters=LBRMoveGroupMixin.params_rviz(
            moveit_configs=moveit_configs_builder.to_moveit_configs()
        )
        + [{"use_sim_time": use_sim_time}],
        remappings=[
            (
                "display_planned_path",
                PathJoinSubstitution([robot_name, "display_planned_path"]),
            ),
            ("joint_states", PathJoinSubstitution([robot_name, "joint_states"])),
            (
                "monitored_planning_scene",
                PathJoinSubstitution([robot_name, "monitored_planning_scene"]),
            ),
            ("planning_scene", PathJoinSubstitution([robot_name, "planning_scene"])),
            (
                "planning_scene_world",
                PathJoinSubstitution([robot_name, "planning_scene_world"]),
            ),
            (
                "robot_description",
                PathJoinSubstitution([robot_name, "robot_description"]),
            ),
            (
                "robot_description_semantic",
                PathJoinSubstitution([robot_name, "robot_description_semantic"]),
            ),
            (
                "recognized_object_array",
                PathJoinSubstitution([robot_name, "recognized_object_array"]),
            ),
        ],
        condition=IfCondition(LaunchConfiguration("rviz")),
    )

    ld.add_action(rviz)
    return ld.entities


def generate_launch_description() -> LaunchDescription:
    ld = LaunchDescription()

    ld.add_action(LBRDescriptionMixin.arg_mode())
    ld.add_action(LBRDescriptionMixin.arg_model())
    ld.add_action(RVizMixin.arg_rviz())

    # depth camera sensor args
    ld.add_action(LBRMoveGroupMixin.arg_sensors_3d())
    ld.add_action(LBRMoveGroupMixin.arg_sensors_3d_config())

    ld.add_action(OpaqueFunction(function=hidden_setup))
    return ld
