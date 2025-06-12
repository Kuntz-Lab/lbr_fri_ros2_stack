#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import Header
    
    
class TwistServoPublisher(Node): # MODIFY NAME
    def __init__(self):
        super().__init__("twist_servo_publisher") # MODIFY NAME

        self.joint_jog_publisher_ = self.create_publisher(
            TwistStamped,
            '/lbr/servo_node/delta_twist_cmds',
            10
        )

        self.pub_timer_ = self.create_timer(
            0.01,  # 100 Hz
            self.publish_twist_jog
        )
    
    def publish_twist_jog(self):
        msg = TwistStamped()

        header_msg = Header()
        header_msg.stamp = self.get_clock().now().to_msg()
        header_msg.frame_id = "lbr_link_ee"

        msg.header = header_msg
        msg.twist.linear.x = -0.05
        msg.twist.linear.y = -0.05
        msg.twist.linear.z = 0.5
        msg.twist.angular.x = 0.0
        msg.twist.angular.y = 0.0
        msg.twist.angular.z = 0.0

        self.joint_jog_publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = TwistServoPublisher() # MODIFY NAME
    rclpy.spin(node)
    rclpy.shutdown()
    
    
if __name__ == "__main__":
    main()