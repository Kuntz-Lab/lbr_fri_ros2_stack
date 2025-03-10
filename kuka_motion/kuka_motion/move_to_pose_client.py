#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.action.client import ClientGoalHandle, GoalStatus
from kuka_interfaces.action import MoveToPose
from geometry_msgs.msg import Vector3
import numpy as np
    
    
class MoveToPoseClientNode(Node): # MODIFY NAME
    def __init__(self):
        super().__init__("move_to_pose_client") # MODIFY NAME
        self.move_to_goal_client_ = ActionClient(
            self, 
            MoveToPose, 
            "move_to_pose")
    

    def send_goal(self, position: Vector3, rpy: Vector3):
        # wait for server to be up
        self.move_to_goal_client_.wait_for_server()

        # create goal
        goal = MoveToPose.Goal()
        goal.desired_pose.header.frame_id = "world"
        goal.desired_pose.header.stamp = self.get_clock().now().to_msg()
        goal.desired_pose.pose.position.x = position.x
        goal.desired_pose.pose.position.y = position.y
        goal.desired_pose.pose.position.z = position.z
        orientation = self.euler_to_quaternion(rpy.x, rpy.y, rpy.z)
        goal.desired_pose.pose.orientation.x = orientation[0]
        goal.desired_pose.pose.orientation.y = orientation[1]
        goal.desired_pose.pose.orientation.z = orientation[2]
        goal.desired_pose.pose.orientation.w = orientation[3]

        # send the goal
        self.get_logger().info("Sending the goal to the MoveToPose server")
        self.move_to_goal_client_.\
            send_goal_async(goal).\
                add_done_callback(self.goal_response_callback)


    def goal_response_callback(self, future):
        self.goal_handle_: ClientGoalHandle = future.result()
        if self.goal_handle_.accepted:
            self.get_logger().info("Goal was accepted by MoveToPose server")
            self.goal_handle_.get_result_async().add_done_callback(self.goal_result_callback)
        else:
            self.get_logger().warn("Goal was rejected by MoveToPose server")


    def goal_result_callback(self, future):

        status = future.result().status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info("Success")
        elif status == GoalStatus.STATUS_ABORTED:
            self.get_logger().error("Aborted")
        elif status == GoalStatus.STATUS_CANCELED:
            self.get_logger().warn("Canceled")

        result = future.result().result
        self.get_logger().info(f"Result: {result}") 


    def euler_to_quaternion(self, roll, pitch, yaw):
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
    node = MoveToPoseClientNode() # MODIFY NAME
    
    # Example end-effector pose
    goal_pos = Vector3()
    goal_pos.x = -0.3
    goal_pos.y = -0.4
    goal_pos.z = 1.0

    goal_rpy = Vector3()
    goal_rpy.x = 0.0
    goal_rpy.y = -0.57
    goal_rpy.z = 0.0

    node.send_goal(goal_pos, goal_rpy)
    rclpy.spin(node)
    rclpy.shutdown()
    
    
if __name__ == "__main__":
    main()