import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
import rclpy.time
from sensor_msgs.msg import JointState
import numpy as np
from tf2_ros import TransformListener, Buffer
from rclpy.executors import MultiThreadedExecutor


class MoveToPoseServerNode(Node):

    ################# INITIALIZE NODE #################

    def __init__(self, robot_name='med14_tc', _sub_echo=False):
        super().__init__('move_to_pose_server')

        # set the flag for which robot we are using (important for what channels to subscribe to/publish to)
        self.robot_name = robot_name
        if self.robot_name == 'med14':
            self.joint_state_topic_name = '/lbr/joint_states'
            self.move_action_name = '/lbr/move_action'
            self.arm_joint_names = ['lbr_A1', 'lbr_A2', 'lbr_A3', 'lbr_A4', 'lbr_A5', 'lbr_A6', 'lbr_A7']
            self.ik_solver_name = '/lbr/compute_ik'
            self.ee_frame = 'lbr_link_7'
            self.base_frame = 'lbr_link_1'
        elif self.robot_name == 'med14_tc':
            self.joint_state_topic_name = '/lbr/joint_states'
            self.move_action_name = '/lbr/move_action'
            self.arm_joint_names = ['lbr_A1', 'lbr_A2', 'lbr_A3', 'lbr_A4', 'lbr_A5', 'lbr_A6', 'lbr_A7']
            self.ik_solver_name = '/lbr/compute_ik'
            self.ee_frame = 'lbr_tendon_robot_link'
            self.base_frame = 'lbr_link_1'
        else:
            self.get_logger().warn('No valid robot name specified in MoveToGoal() initialization, various topics/services/actions may not work!') 
            self.joint_state_topic_name = 'joint_states'

        ######### INITIALIZE PROGRAM VARIABLES #########
        # set subscriber echo flag
        self._sub_echo = _sub_echo

        # Create a TF buffer and listener, objects for storing joint state and ee pose
        self.tf_buffer = Buffer() 
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.ee_pose = Pose()
        self.medbot_joint_state = JointState()

        # create timer for executing ee pose update callback
        self.tf_update_timer = self.create_timer(
            0.01,       # update at 100 hz
            self.ee_pose_listener_callback
            )
        self.get_logger().info("End-effector pose listener has been started")

    
    ################# DEFINE CALLBACKS #################

    def ee_pose_listener_callback(self):
        try: 
            # Lookup transform from base_link to end_effector 
            now = rclpy.time.Time()
            
            transform = self.tf_buffer.lookup_transform(self.base_frame, self.ee_frame, now) 
            
            # Extract translation and rotation from the transform 
            translation = transform.transform.translation 
            rotation = transform.transform.rotation             

            self.ee_pose.position.x = translation.x
            self.ee_pose.position.y = translation.y
            self.ee_pose.position.z = translation.z

            self.ee_pose.orientation.x = rotation.x
            self.ee_pose.orientation.y = rotation.y
            self.ee_pose.orientation.z = rotation.z
            self.ee_pose.orientation.w = rotation.w

            if self._sub_echo:
                self.get_logger().info(f"\nEnd Effector Position: x={translation.x}, y={translation.y}, z={translation.z}\nEnd Effector Orientation: x={rotation.x}, y={rotation.y}, z={rotation.z}, w={rotation.w}") 
            
        except Exception as e: 
            self.get_logger().warn(f"Could not get transform: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = MoveToPoseServerNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()
    rclpy.shutdown()



if __name__ == '__main__':
    main()
