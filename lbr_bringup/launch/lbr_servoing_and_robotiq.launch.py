from launch import LaunchDescription
from launch.actions import RegisterEventHandler, DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessStart
from launch.substitutions import LaunchConfiguration, PythonExpression, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

def generate_launch_description() -> LaunchDescription:

    # init launch description
    ld = LaunchDescription()

    # Launch Arguments
    model_arg = DeclareLaunchArgument(
        'model',
        default_value='med14_robotiq_2f',
        description='Robot model to use'
    )
    ld = LaunchDescription()
    ld.add_action(model_arg)
    model = LaunchConfiguration('model')

    mode_arg = DeclareLaunchArgument(
        'mode',
        default_value='mock',
        description='Robot operation mode: mock, hardware'
    )
    ld.add_action(mode_arg)
    mode = LaunchConfiguration('mode')

    rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description='Whether to start RViz'
    )
    ld.add_action(rviz_arg)
    rviz = LaunchConfiguration('rviz')

    # Package paths
    lbr_bringup_path = FindPackageShare('lbr_bringup')]

    # If the model is med14_robotiq_2f, use the combined controllers cfg file, otherwise use the default    #ADDED
    ctrl_cfg = PythonExpression([
        "'ros2_control/combined_controllers.yaml' if '", model, "' == 'med14_robotiq_2f' else 'ros2_control/lbr_controllers.yaml'"
    ])    

    # Include mock.launch.py with parameters
    mock_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([lbr_bringup_path, 'launch', 'mock.launch.py'])
        ]),
        launch_arguments={
            'ctrl': 'forward_position_controller',
            'model': model,
            'ctrl_cfg_pkg': 'lbr_description',   #ADDED
            'ctrl_cfg': ctrl_cfg, #'ros2_control/combined_controllers.yaml'  # Use combined controllers  #ADDED
            'use_fake_hardware': 'true'  # IMPORTANT: Use fake hardware for mock mode    #ADDED
        }.items(),
        condition=IfCondition(PythonExpression(["'", mode, "' == 'mock' or '", mode, "' == 'gazebo'"]))
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


    # Include moveit_servo.launch.py with parameters
    moveit_servo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([lbr_bringup_path, 'launch', 'moveit_servo.launch.py'])
        ]),
        launch_arguments={
            'mode': mode,
            'model': model,
        }.items(),
    )
    ld.add_action(moveit_servo_launch)

    # Conditionally include rviz.launch.py
    rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([lbr_bringup_path, 'launch', 'rviz.launch.py'])
        ]),
        condition=IfCondition(rviz)
    )
    ld.add_action(rviz_launch)

    return ld





