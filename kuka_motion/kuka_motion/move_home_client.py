#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.action.client import ClientGoalHandle, GoalStatus
from kuka_interfaces.action import MoveHome
from geometry_msgs.msg import Vector3
import numpy as np
    
    
class MoveHomeClientNode(Node): # MODIFY NAME
    def __init__(self):
        super().__init__("move_to_pose_client") # MODIFY NAME
        self.move_to_goal_client_ = ActionClient(
            self, 
            MoveHome, 
            "move_to_home")
    

    def send_home_command(self, 
                  planning_group: str = "arm",
                  vel_scaling: float = 0.1,
                  acc_scaling: float = 0.1):
        # wait for server to be up
        self.get_logger().info("Waiting for MoveHome server to be up")
        while not self.move_to_goal_client_.wait_for_server(timeout_sec=1.0):
            self.get_logger().info("MoveHome server not available, waiting again...")
        self.get_logger().info("MoveHome server is up")

        # create goal
        goal = MoveHome.Goal()

        # add planning group
        goal.planning_group = planning_group
        goal.vel_scaling = vel_scaling
        goal.acc_scaling = acc_scaling

        # send the goal
        self.get_logger().info("Sending the goal to the MoveHome server")
        self.move_to_goal_client_.\
            send_goal_async(goal).\
                add_done_callback(self.goal_response_callback)


    def goal_response_callback(self, future):
        self.goal_handle_: ClientGoalHandle = future.result()
        if self.goal_handle_.accepted:
            self.get_logger().info("Goal was accepted by MoveHome server")
            self.goal_handle_.get_result_async().add_done_callback(self.goal_result_callback)
        else:
            self.get_logger().warn("Goal was rejected by MoveHome server")


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
        rclpy.shutdown()



def main(args=None):
    rclpy.init(args=args)
    node = MoveHomeClientNode() # MODIFY NAME

    node.send_home_command("arm", 0.1, 0.1)
    rclpy.spin(node)
    # rclpy.shutdown()
    
    
if __name__ == "__main__":
    main()