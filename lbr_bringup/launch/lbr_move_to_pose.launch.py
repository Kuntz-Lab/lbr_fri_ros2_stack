from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, OpaqueFunction, DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    ld = LaunchDescription()

    # Launch Arguments
    model_arg = DeclareLaunchArgument(
        'model',
        default_value='med14_tc',
        description='Robot model to use'
    )
    ld.add_action(model_arg)
    model = LaunchConfiguration('model')

    rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description='Whether to start RViz'
    )
    ld.add_action(rviz_arg)
    rviz = LaunchConfiguration('rviz')

    mode_arg = DeclareLaunchArgument(
        'mode',
        default_value='mock',
        description='Robot operation mode: mock, hardware'
    )
    ld.add_action(mode_arg)
    mode = LaunchConfiguration('mode')

    # pointcloud moveit arguements
    sensors_3d_arg = DeclareLaunchArgument(
        'sensors_3d',
        default_value='false',
        description='Whether to use 3D sensors for collision detection'
    )
    ld.add_action(sensors_3d_arg)
    sensors_3d = LaunchConfiguration('sensors_3d')

    sensors_3d_config_arg = DeclareLaunchArgument(
        'sensors_3d_config',
        default_value='',
        description='Path to the 3D sensor configuration file'
    )
    ld.add_action(sensors_3d_config_arg)    
    sensors_3d_config = LaunchConfiguration('sensors_3d_config')

    ############################################
    # MOCK LAUNCH
    ############################################
    # If the model is med14_robotiq_2f, use the combined controllers cfg file, otherwise use the default
    ctrl_cfg = PythonExpression([
        "'ros2_control/combined_controllers.yaml' if '", model, "' == 'med14_robotiq_2f' else 'ros2_control/lbr_controllers.yaml'"
    ])    

    # Include mock.launch.py to launch the robot description and sim robot controllers conditionally.
    # This is done with the hardware launch file on the robot pc if the harware mode is being used.
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
            'ctrl_cfg': ctrl_cfg, #'ros2_control/combined_controllers.yaml'  # Use combined controllers
            'use_fake_hardware': 'true'  # IMPORTANT: Use fake hardware for mock mode
        }.items()
        ,
        condition=IfCondition(
            PythonExpression([
                "'", mode, "' == 'gazebo' or '", mode, "' == 'mock'"
            ])
        )
    )
    ld.add_action(mock_launch)

    ############################################
    # GRIPPER CONTROLLER SPAWNER
    ############################################
    robotiq_gripper_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['robotiq_gripper_controller', '--controller-manager', 'controller_manager'],
        namespace='lbr',
        # condition=IfCondition(PythonExpression([
        #     "'true' if ('", model, "' == 'med14_robotiq_2f') and ('", mode, "' == 'gazebo' or '", mode, "' == 'mock') else 'false'"
        # ]))
        # condition=IfCondition(PythonExpression(["'true' if '", model, "' == 'med14_robotiq_2f' else 'false'"]))
        # condition=IfCondition(PythonExpression(["'", model, "' == 'med14_robotiq_2f'"]))
        condition=IfCondition(PythonExpression(["('", model, "' == 'med14_robotiq_2f') and (('", mode, "' == 'gazebo') or ('", mode, "' == 'mock'))"]))
    )
    ld.add_action(robotiq_gripper_controller_spawner)
                
    robotiq_activation_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['robotiq_activation_controller', '--controller-manager', 'controller_manager'],
        namespace='lbr',
        # condition=IfCondition(PythonExpression([
        #     "'true' if ('", model, "' == 'med14_robotiq_2f') and ('", mode, "' == 'gazebo' or '", mode, "' == 'mock') else 'false'"
        # ]))
        condition=IfCondition(PythonExpression(["('", model, "' == 'med14_robotiq_2f') and (('", mode, "' == 'gazebo') or ('", mode, "' == 'mock'))"]))
        # condition=IfCondition(PythonExpression(["'", model, "' == 'med14_robotiq_2f'"]))
        # condition=IfCondition(PythonExpression(["'true' if '", model, "' == 'med14_robotiq_2f' else 'false'"]))

    )
    ld.add_action(robotiq_activation_controller_spawner)

    ############################################
    # MOVE-IT
    ############################################
    # Include move_group.launch.py
    def print_launch_args(context, *args, **kwargs):
        resolved_model = context.launch_configurations.get('model')
        resolved_mode = context.launch_configurations.get('mode')
        resolved_rviz = context.launch_configurations.get('rviz')
        resolved_sensors_3d = context.launch_configurations.get('sensors_3d')
        resolved_sensors_3d_config = context.launch_configurations.get('sensors_3d_config')
        
        print("\n"*20)
        print(f"Model: {resolved_model}")
        print(f"Mode: {resolved_mode}")
        print(f"RViz: {resolved_rviz}")
        print(f"Sensors 3D: {resolved_sensors_3d}")
        print(f"Sensors 3D Config: {resolved_sensors_3d_config}")
        print("\n"*20)
        
        # Must return actions to be executed
        return []

    # Add this before move_group_launch
    print_args = OpaqueFunction(function=print_launch_args)
    ld.add_action(print_args)

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
            'sensors_3d': sensors_3d,
            'sensors_3d_config': sensors_3d_config
        }.items()
    )
    ld.add_action(move_group_launch)

    ############################################
    # MOVE TO POSE SERVER
    ############################################
    # Launch move_to_pose server
    move_to_pose_server = Node(
        package='lbr_motion',
        executable='move_to_pose_server',
        name='move_to_pose_server',
        output='screen',
        parameters=[{'robot_name': PythonExpression(["'", model, "'"])}]
    )
    ld.add_action(move_to_pose_server)

    ############################################
    # MOVE HOME SERVER
    ############################################
    # Launch move_home server
    move_home_server = Node(
        package='lbr_motion',
        executable='move_home_server',
        name='move_home_server',
        output='screen',
        parameters=[{'robot_name': PythonExpression(["'", model, "'"])}]
    )
    ld.add_action(move_home_server)

    return ld