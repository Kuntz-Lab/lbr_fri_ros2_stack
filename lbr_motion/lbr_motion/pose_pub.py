#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Pose
from std_msgs.msg import Header
import numpy as np
    
    
class PosePublisher(Node): # MODIFY NAME
    def __init__(self):
        super().__init__("twist_servo_publisher") # MODIFY NAME

        self._desired_pose_publisher = self.create_publisher(
            PoseStamped,
            '/lbr/move_to_pose/desired_pose',
            10
        )

        # # Uncomment to publish on timer
        # self.pub_timer_ = self.create_timer(
        #     0.25,
        #     self.publish_desired_pose
        # )

        # Uncomment to only publish once
        # self.publish_desired_pose()


    def publish_desired_pose(self, pose: Pose, frame_id: str = "world"):
        msg = PoseStamped()

        header_msg = Header()
        header_msg.stamp = self.get_clock().now().to_msg()
        header_msg.frame_id = frame_id

        msg.header = header_msg
        msg.pose = pose

        self._desired_pose_publisher.publish(msg)

def euler_to_quaternion(roll, pitch, yaw):
    """Converts Euler angles (in radians) to a quaternion."""
    cy = np.cos(yaw * 0.5)
    sy = np.sin(yaw * 0.5)
    cp = np.cos(pitch * 0.5)
    sp = np.sin(pitch * 0.5)
    cr = np.cos(roll * 0.5)
    sr = np.sin(roll * 0.5)

    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy

    return [x, y, z, w]

def main(args=None):
    rclpy.init(args=args)
    node = PosePublisher() # MODIFY NAME

    goal_pose = Pose()
    goal_pose.position.x = 0.0
    goal_pose.position.y = 0.0
    goal_pose.position.z = 1.3

    # since quaternions can be a bit tricky to specify, we will use Euler angles for the example. All real ROS applications use strictly quaternions and rotation matrices.
    # roll, pitch, yaw are in radians
    roll = 0.0
    pitch = 0.0
    yaw = 0.0
    goal_quat = euler_to_quaternion(roll, pitch, yaw)
    goal_pose.orientation.x = goal_quat[0]
    goal_pose.orientation.y = goal_quat[1]
    goal_pose.orientation.z = goal_quat[2]
    goal_pose.orientation.w = goal_quat[3]

    frame_id = "world"

    node.publish_desired_pose(goal_pose, frame_id)
    rclpy.spin(node)
    rclpy.shutdown()
    
    
if __name__ == "__main__":
    main()