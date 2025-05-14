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

    # Include hardware.launch.py
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

    # Add the gripper controller spawners (these will run after the hardware has started)
    # They're conditionally executed only when the model is med14_robotiq_2f
    robotiq_gripper_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['robotiq_gripper_controller', '--controller-manager', 'controller_manager'],
        namespace=robot_name,
        condition=IfCondition(PythonExpression(["'", model, "' == 'med14_robotiq_2f'"]))
    )
    ld.add_action(robotiq_gripper_controller_spawner)
    
    robotiq_activation_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['robotiq_activation_controller', '--controller-manager', 'controller_manager'],
        namespace=robot_name,
        condition=IfCondition(PythonExpression(["'", model, "' == 'med14_robotiq_2f'"]))
    )
    ld.add_action(robotiq_activation_controller_spawner)

    return ld

# def generate_launch_description() -> LaunchDescription:
#     ld = LaunchDescription()

#     # launch arguments
#     ld.add_action(LBRDescriptionMixin.arg_model())
#     ld.add_action(LBRDescriptionMixin.arg_robot_name())
#     ld.add_action(LBRROS2ControlMixin.arg_sys_cfg_pkg())
#     ld.add_action(LBRROS2ControlMixin.arg_sys_cfg())
#     ld.add_action(LBRROS2ControlMixin.arg_ctrl_cfg_pkg())
#     ld.add_action(LBRROS2ControlMixin.arg_ctrl_cfg())
#     ld.add_action(LBRROS2ControlMixin.arg_ctrl())

#     # Add COM port argument for the gripper
#     com_port_arg = DeclareLaunchArgument(
#         'com_port',
#         default_value='/dev/ttyUSB1',
#         description='Serial port for the Robotiq gripper'
#     )
#     ld.add_action(com_port_arg)
#     com_port = LaunchConfiguration('com_port')
#     model = LaunchConfiguration('model')

#     # static transform world -> <robot_name>_floating_link
#     ld.add_action(
#         LBRDescriptionMixin.node_static_tf(
#             tf=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
#             parent="world",
#             child=PythonExpression(
#                 ["'", LaunchConfiguration("robot_name"), "' + '_floating_link'"]
#             ),
#         )
#     )

#     # robot description
#     robot_description = LBRDescriptionMixin.param_robot_description(mode="hardware",model=model,com_port=com_port)

#     # robot state publisher
#     robot_state_publisher = LBRROS2ControlMixin.node_robot_state_publisher(
#         robot_description=robot_description, use_sim_time=False
#     )
#     ld.add_action(robot_state_publisher)

#     # ros2 control node
#     ros2_control_node = LBRROS2ControlMixin.node_ros2_control(
#         use_sim_time=False, robot_description=robot_description
#     )
#     ld.add_action(ros2_control_node)

#     # joint state broad caster and controller on ros2 control node start
#     joint_state_broadcaster = LBRROS2ControlMixin.node_controller_spawner(
#         controller="joint_state_broadcaster"
#     )
#     force_torque_broadcaster = LBRROS2ControlMixin.node_controller_spawner(
#         controller="force_torque_broadcaster"
#     )
#     lbr_state_broadcaster = LBRROS2ControlMixin.node_controller_spawner(
#         controller="lbr_state_broadcaster"
#     )
#     controller = LBRROS2ControlMixin.node_controller_spawner(
#         controller=LaunchConfiguration("ctrl")
#     )

#     # Add the gripper controller spawners
#     robotiq_gripper_controller_spawner = Node(
#         package='controller_manager',
#         executable='spawner',
#         arguments=['robotiq_gripper_controller', '--controller-manager', 'controller_manager'],
#         namespace=LaunchConfiguration('robot_name'),
#         condition=IfCondition(PythonExpression(["'", model, "' == 'med14_robotiq_2f'"]))
#     )
    
#     robotiq_activation_controller_spawner = Node(
#         package='controller_manager',
#         executable='spawner',
#         arguments=['robotiq_activation_controller', '--controller-manager', 'controller_manager'],
#         namespace=LaunchConfiguration('robot_name'),
#         condition=IfCondition(PythonExpression(["'", model, "' == 'med14_robotiq_2f'"]))
#     )

#     controller_event_handler = RegisterEventHandler(
#         OnProcessStart(
#             target_action=ros2_control_node,
#             on_start=[
#                 joint_state_broadcaster,
#                 force_torque_broadcaster,
#                 lbr_state_broadcaster,
#                 controller,
#                 robotiq_gripper_controller_spawner,
#                 robotiq_activation_controller_spawner,
#             ],
#         )
#     )
#     ld.add_action(controller_event_handler)
#     return ld


