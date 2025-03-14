#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from control_msgs.msg import JointJog
from std_msgs.msg import Header
    
    
class JointServoPublisher(Node): # MODIFY NAME
    def __init__(self):
        super().__init__("joint_servo_publisher") # MODIFY NAME

        self.joint_jog_publisher_ = self.create_publisher(
            JointJog,
            '/lbr/servo_node/delta_joint_cmds',
            10
        )

        self.pub_timer_ = self.create_timer(
            0.250,
            self.publish_joint_jog
        )
    
    def publish_joint_jog(self):
        msg = JointJog()

        header_msg = Header()
        header_msg.stamp = self.get_clock().now().to_msg()
        header_msg.frame_id = "base_link"

        msg.header = header_msg
        msg.joint_names = ["lbr_A2", "lbr_A4", "lbr_A6"]
        msg.displacements = [2.0,  -4.0, 2.0]
        msg.velocities = [2.0, 4.0, 2.0]
        msg.duration = 0.250

        self.joint_jog_publisher_.publish(msg)



def main(args=None):
    rclpy.init(args=args)
    node = JointServoPublisher() # MODIFY NAME
    rclpy.spin(node)
    rclpy.shutdown()
    
    
if __name__ == "__main__":
    main()