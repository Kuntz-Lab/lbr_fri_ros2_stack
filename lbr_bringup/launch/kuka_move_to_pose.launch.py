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

    # Include mock.launch.py conditionally
    mock_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('lbr_bringup'),
                'launch',
                'mock.launch.py'
            ])
        ]),
        launch_arguments={
            'model': model
        }.items(),
        condition=IfCondition(
            PythonExpression([
                "'", mode, "' == 'gazebo' or '", mode, "' == 'mock'"
            ])
        )
    )
    ld.add_action(mock_launch)

    # # if mode == 'gazebo' or mode == 'mock':
    # # Include mock.launch.py
    # mock_launch = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource([
    #         PathJoinSubstitution([
    #             FindPackageShare('lbr_bringup'),
    #             'launch',
    #             'mock.launch.py'
    #         ])
    #     ]),
    #     launch_arguments={
    #         'model': model
    #     }.items()
    # )
    # ld.add_action(mock_launch)

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
            'model': model
        }.items()
    )
    ld.add_action(move_group_launch)

    # Launch move_to_pose node
    move_to_pose_node = Node(
        package='kuka_motion',
        executable='move_to_pose',
        name='move_to_pose',
        output='screen',
        parameters=[{'robot_name': model}]
    )
    ld.add_action(move_to_pose_node)

    return ld