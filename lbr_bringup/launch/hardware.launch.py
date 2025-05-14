from launch import LaunchDescription
from launch.actions import RegisterEventHandler, DeclareLaunchArgument
from launch.event_handlers import OnProcessStart
from launch.substitutions import LaunchConfiguration, PythonExpression
from lbr_bringup.description import LBRDescriptionMixin
from lbr_bringup.ros2_control import LBRROS2ControlMixin
from launch_ros.actions import Node
from launch.conditions import IfCondition


def generate_launch_description() -> LaunchDescription:
    ld = LaunchDescription()

    # launch arguments
    ld.add_action(LBRDescriptionMixin.arg_model())
    ld.add_action(LBRDescriptionMixin.arg_robot_name())
    ld.add_action(LBRROS2ControlMixin.arg_sys_cfg_pkg())
    ld.add_action(LBRROS2ControlMixin.arg_sys_cfg())
    ld.add_action(LBRROS2ControlMixin.arg_ctrl_cfg_pkg())
    ld.add_action(LBRROS2ControlMixin.arg_ctrl_cfg())
    ld.add_action(LBRROS2ControlMixin.arg_ctrl())

    # Add COM port argument for the gripper
    com_port_arg = DeclareLaunchArgument(
        'com_port',
        default_value='/dev/ttyUSB1',
        description='Serial port for the Robotiq gripper'
    )
    ld.add_action(com_port_arg)
    com_port = LaunchConfiguration('com_port')
    model = LaunchConfiguration('model')

    # static transform world -> <robot_name>_floating_link
    ld.add_action(
        LBRDescriptionMixin.node_static_tf(
            tf=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            parent="world",
            child=PythonExpression(
                ["'", LaunchConfiguration("robot_name"), "' + '_floating_link'"]
            ),
        )
    )

    # robot description
    robot_description = LBRDescriptionMixin.param_robot_description(mode="hardware",model=model,com_port=com_port)

    # robot state publisher
    robot_state_publisher = LBRROS2ControlMixin.node_robot_state_publisher(
        robot_description=robot_description, use_sim_time=False
    )
    ld.add_action(robot_state_publisher)

    # ros2 control node
    ros2_control_node = LBRROS2ControlMixin.node_ros2_control(
        use_sim_time=False, robot_description=robot_description
    )
    ld.add_action(ros2_control_node)

    # joint state broad caster and controller on ros2 control node start
    joint_state_broadcaster = LBRROS2ControlMixin.node_controller_spawner(
        controller="joint_state_broadcaster"
    )
    force_torque_broadcaster = LBRROS2ControlMixin.node_controller_spawner(
        controller="force_torque_broadcaster"
    )
    lbr_state_broadcaster = LBRROS2ControlMixin.node_controller_spawner(
        controller="lbr_state_broadcaster"
    )
    controller = LBRROS2ControlMixin.node_controller_spawner(
        controller=LaunchConfiguration("ctrl")
    )

    # Add the gripper controller spawners
    robotiq_gripper_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['robotiq_gripper_controller', '--controller-manager', 'controller_manager'],
        namespace=LaunchConfiguration('robot_name'),
        condition=IfCondition(PythonExpression(["'", model, "' == 'med14_robotiq_2f'"]))
    )
    
    robotiq_activation_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['robotiq_activation_controller', '--controller-manager', 'controller_manager'],
        namespace=LaunchConfiguration('robot_name'),
        condition=IfCondition(PythonExpression(["'", model, "' == 'med14_robotiq_2f'"]))
    )

    controller_event_handler = RegisterEventHandler(
        OnProcessStart(
            target_action=ros2_control_node,
            on_start=[
                joint_state_broadcaster,
                force_torque_broadcaster,
                lbr_state_broadcaster,
                controller,
                robotiq_gripper_controller_spawner,
                robotiq_activation_controller_spawner,
            ],
        )
    )
    ld.add_action(controller_event_handler)
    return ld


