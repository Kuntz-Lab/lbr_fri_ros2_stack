from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    ld = LaunchDescription()

    # Launch Arguments
    model = LaunchConfiguration('model', default='med14_tc')
    rviz = LaunchConfiguration('rviz', default='true')
    mode = LaunchConfiguration('mode', default='mock')

    ############################################
    # MOCK OR HARDWARE LAUNCH
    ############################################
    # Include mock.launch.py conditionally
    # if mode == 'mock' or mode == 'gazebo':
    mock_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('lbr_bringup'),
                'launch',
                'mock.launch.py'
            ])
        ]),
        launch_arguments={
            'model': model,
            'ctrl_cfg_pkg': 'lbr_description',
            'ctrl_cfg': 'ros2_control/combined_controllers.yaml'  # Use combined controllers
        }.items()
        ,
        condition=IfCondition(
            PythonExpression([
                "'", mode, "' == 'gazebo' or '", mode, "' == 'mock'"
            ])
        )
    )
    ld.add_action(mock_launch)

    robotiq_gripper_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['robotiq_gripper_controller', '--controller-manager', 'controller_manager'],
        namespace='lbr',
        condition=IfCondition(PythonExpression(["'", mode, "' == 'mock'"]))
    )
    ld.add_action(robotiq_gripper_controller_spawner)
                
    robotiq_activation_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['robotiq_activation_controller', '--controller-manager', 'controller_manager'],
        namespace='lbr',
        condition=IfCondition(PythonExpression(["'", mode, "' == 'mock'"]))
    )
    ld.add_action(robotiq_activation_controller_spawner)

    ############################################
    # RVIZ
    ############################################
    # Include RViz if rviz is true
    rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('lbr_bringup'),
                'launch',
                'rviz.launch.py'
            ])
        ]),
        launch_arguments={
            'model': model
        }.items()
    )
    ld.add_action(rviz_launch)

    ############################################
    # MOVE-IT
    ############################################
    # Include move_group.launch.py
    move_group_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('lbr_bringup'),
                'launch',
                'move_group.launch.py'
            ])
        ]),
        launch_arguments={
            'mode': mode,
            'rviz': rviz,
            'model': model,
            # 'ctrl_cfg': PathJoinSubstitution([
            #     FindPackageShare('med14_robotiq_2f_moveit_config'),
            #     'config', 
            #     'lbr_robotiq_controllers.yaml'
            # ])  # Pass the robotiq controllers YAML
        }.items()
    )
    ld.add_action(move_group_launch)
    
    ############################################
    # MOVE TO POSE
    ############################################
    # Launch move_to_pose node
    # move_to_pose_node = Node(
    #     package='kuka_motion',
    #     executable='move_to_pose',
    #     name='move_to_pose',
    #     output='screen',
    #     parameters=[{'robot_name': model}]
    # )
    # ld.add_action(move_to_pose_node)

    return ld