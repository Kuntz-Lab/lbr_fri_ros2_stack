from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnProcessStart
from launch.substitutions import LaunchConfiguration, PythonExpression, PathJoinSubstitution
from lbr_bringup.description import LBRDescriptionMixin
from lbr_bringup.ros2_control import LBRROS2ControlMixin
from launch_ros.actions import Node
from launch.conditions import IfCondition
from launch_ros.substitutions import FindPackageShare
import launch_ros


def generate_launch_description() -> LaunchDescription:
    ld = LaunchDescription()

    # Launch Arguments
    model_arg = DeclareLaunchArgument(
        'model',
        default_value='med14_robotiq_2f',
        description='Robot model to use'
    )
    ld.add_action(model_arg)
    model = LaunchConfiguration('model')

    robot_name_arg = DeclareLaunchArgument(
        'robot_name',
        default_value='lbr',
        description='Robot namespace'
    )
    ld.add_action(robot_name_arg)
    robot_name = LaunchConfiguration('robot_name')

    com_port_arg = DeclareLaunchArgument(
        'com_port',
        default_value='/dev/ttyUSB1',
        description='Serial port for the Robotiq gripper'
    )
    ld.add_action(com_port_arg)
    com_port = LaunchConfiguration('com_port')

    # Select the right controller configuration based on robot model
    ctrl_cfg = PythonExpression([
        "'ros2_control/combined_controllers.yaml' if '", model, "' == 'med14_robotiq_2f' else 'ros2_control/lbr_controllers.yaml'"
    ])

    # Include hardware.launch.py for the KUKA lbr robot
    hardware_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('lbr_bringup'),
                'launch',
                'hardware.launch.py'
            ])
        ]),
        launch_arguments={
            'model': model,
            'robot_name': robot_name,
            'ctrl_cfg': ctrl_cfg,
            'com_port': com_port
        }.items()
    )
    ld.add_action(hardware_launch)

    # Spawn the robotiq gripper controllers
    robotiq_gripper_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        namespace=robot_name,
        arguments=[
            "robotiq_gripper_controller", 
            "--controller-manager", 
            "controller_manager"
        ],
    )
    ld.add_action(robotiq_gripper_controller_spawner)
    
    robotiq_activation_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        namespace=robot_name,
        arguments=[
            "robotiq_activation_controller", 
            "--controller-manager", 
            "controller_manager"
        ],
    )
    ld.add_action(robotiq_activation_controller_spawner)

    return ld




