#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.action.client import ClientGoalHandle, GoalStatus
from lbr_interfaces.action import MoveToPose
from geometry_msgs.msg import Vector3, Pose
import numpy as np
    
    
class MoveToPoseClientNode(Node): # MODIFY NAME
    def __init__(self):
        super().__init__("move_to_pose_client") # MODIFY NAME
        self.move_to_goal_client_ = ActionClient(
            self, 
            MoveToPose, 
            "move_to_pose")
    

    def send_goal(self, 
                  pose: Pose, 
                  planning_group: str = "arm",
                  vel_scaling: float = 0.1,
                  acc_scaling: float = 0.1):
        # wait for server to be up
        self.move_to_goal_client_.wait_for_server()

        # create goal
        goal = MoveToPose.Goal()

        # add header
        goal.desired_pose.header.frame_id = "world"
        goal.desired_pose.header.stamp = self.get_clock().now().to_msg()

        # add pose
        goal.desired_pose.pose = pose

        # add planning group
        goal.planning_group = planning_group
        goal.vel_scaling = vel_scaling
        goal.acc_scaling = acc_scaling

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
    node = MoveToPoseClientNode() # MODIFY NAME
    
    # Example end-effector pose
    goal_pose = Pose()
    goal_pose.position.x = 0.0
    goal_pose.position.y = 0.0
    goal_pose.position.z = 1.3
    # since it can be a bit tricky to specify quaternions, we will use Euler angles for the example. All real ROS applications use strictly quaternions and rotation matrices.
    # roll, pitch, yaw are in radians
    roll = 0.0
    pitch = 0.0
    yaw = 0.0
    goal_quat = euler_to_quaternion(roll, pitch, yaw)
    goal_pose.orientation.x = goal_quat[0]
    goal_pose.orientation.y = goal_quat[1]
    goal_pose.orientation.z = goal_quat[2]  
    goal_pose.orientation.w = goal_quat[3]

    node.send_goal(goal_pose, "arm", 0.1, 0.1)
    rclpy.spin(node)
    rclpy.shutdown()
    
    
if __name__ == "__main__":
    main()