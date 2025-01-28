import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.action.client import ClientGoalHandle, GoalStatus
from moveit_msgs.action import MoveGroup
from geometry_msgs.msg import Pose, Vector3, PoseStamped
from moveit_msgs.msg import Constraints, JointConstraint, AttachedCollisionObject, CollisionObject
import rclpy.time
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from shape_msgs.msg import SolidPrimitive
from moveit_msgs.srv import GetPositionIK
from tf2_ros import TransformListener, Buffer
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from enum import Enum
from threading import Lock

class NodeState(Enum):
    IDLE = 0
    PLANNING = 1
    EXECUTING = 2


class MoveToPoseNode(Node):

    ################# INITIALIZE NODE #################

    def __init__(self, robot_name='med14_tc', _sub_echo=False):
        super().__init__('move_to_pose')

        ######### INITIALIZE PROGRAM VARIABLES #########

        # State management
        self.state = NodeState.IDLE
        self.state_lock = Lock()
        # for handling goal states and action server/client
        # self.move_to_pose_goal_handle_: ServerGoalHandle = None
        # self.move_to_pose_goal_lock_ = threading.Lock()
        self.move_action_executing_ = False
        self.move_action_goal_handle_: ClientGoalHandle = None
        self.cb_group = ReentrantCallbackGroup()


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

        # set subscriber echo flag
        self._sub_echo = _sub_echo

        # set the workspace bounds
        self.ws_bounds = (Vector3(x=-1.0,y=-1.0,z=-1.0), Vector3(x=1.0,y=1.0,z=1.0))

        # Create a TF buffer and listener, objects for storing joint state and ee pose
        self.tf_buffer = Buffer() 
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.ee_pose = Pose()
        self.medbot_joint_state = JointState()

        ######### INITIALIZE ROS NODES #########
        # Create desired pose subscriber
        self.pose_sub = self.create_subscription(
            PoseStamped,
            '/lbr/move_to_pose/desired_pose',
            self.pose_sub_callback,
            10,
            callback_group=self.cb_group
        )

        # create joint state subscriber
        self.joint_state_sub_ = self.create_subscription(
            JointState,
            self.joint_state_topic_name,
            self.joint_state_listener_callback,
            100     # the topic publishes at approx. 200 hz so can go up to that if needed
            )
        self.get_logger().info("MoveToPose joint state subscriber has been started")

        # create timer for executing ee pose update callback
        self.tf_update_timer = self.create_timer(
            0.01,       # update at 100 hz
            self.ee_pose_listener_callback
            )
        self.get_logger().info("End-effector pose listener has been started")

        # create the action client for sending the planning and execution request to moveit
        self.move_action_client = ActionClient(
            self, 
            MoveGroup, 
            self.move_action_name,
            callback_group=self.cb_group
            )

        # create a client for the GetPositionIK service
        self.ik_service_client = self.create_client(
            GetPositionIK, 
            self.ik_solver_name,
            callback_group=self.cb_group
            )

        # wait for move_group to be started
        while not self.ik_service_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('MoveGroup services not available, waiting...')
        self.get_logger().info("MoveGroup services have been started")

    
    ################# DEFINE CALLBACKS #################

    async def pose_sub_callback(self, msg):
        """Non-blocking pose callback"""
        
        with self.state_lock:
            if self.state != NodeState.IDLE:
                self.get_logger().warn("Node busy, rejecting new goal")
                return
            self.state = NodeState.PLANNING
        
        try:
            # Request IK solution asynchronously
            joint_positions = await self.convert_pose_to_joint_angles(msg)
            if not joint_positions:
                raise Exception("No IK solution found")
                
            # Create and send goal
            success = await self.execute_move(joint_positions)
            if not success:
                raise Exception("Move execution failed")
                
        except Exception as e:
            self.get_logger().error(f"Move failed: {e}")
            
        finally:
            with self.state_lock:
                self.state = NodeState.IDLE

    async def get_ik_solution(self, pose_msg):
        """Non-blocking IK service call"""
        request = GetPositionIK.Request()
        request.ik_request.pose_stamped = pose_msg
        
        try:
            response = await self.ik_service_client.call_async(request)
            if response.error_code.val == response.error_code.SUCCESS:
                return response.solution.joint_state.position
        except Exception as e:
            self.get_logger().error(f"IK failed: {e}")
        return None

    async def execute_move(self, joint_positions):
        """Non-blocking move execution"""
        with self.state_lock:
            self.state = NodeState.EXECUTING
            
        try:
            # Create goal
            goal_msg = self.create_move_group_goal(joint_positions)
            
            # Send goal
            goal_handle = await self.move_action_client.send_goal_async(goal_msg)
            if not goal_handle.accepted:
                return False
                
            # Wait for result
            result = await goal_handle.get_result_async()
            return result.status == GoalStatus.STATUS_SUCCEEDED
            
        finally:
            with self.state_lock:
                self.state = NodeState.IDLE

    # def pose_sub_callback(self, msg: PoseStamped):
    #     """Callback when new pose is received"""
    #     self.get_logger().info('Received new pose')
        
    #     # Skip if another move is in progress
    #     if self.move_action_executing_:
    #         self.get_logger().warn("Already executing a move, skipping new goal")
    #         return

    #     self.move_action_executing_ = True
    #     self.execute_move(msg)

    # def execute_move(self, goal_pose):
    #     """Execute move to pose asynchronously"""
    #     try:
    #         planning_group = 'arm'
            
    #         # Find IK solution
    #         self.get_logger().info("Finding IK solution")
    #         joint_positions = self.convert_pose_to_joint_angles(goal_pose, planning_group)

    #         if joint_positions is None:
    #             self.get_logger().error("No valid IK solution found")
    #             self.move_action_executing_ = False
    #             return

    #         # Create and send goal
    #         move_goal = self.create_move_group_goal(joint_positions, planning_group)
    #         self.move_action_client.wait_for_server()
            
    #         send_goal_future = self.move_action_client.send_goal_async(move_goal)
    #         send_goal_future.add_done_callback(self.goal_response_callback)

    #     except Exception as e:
    #         self.get_logger().error(f"Failed to execute move: {e}")
    #         self.move_action_executing_ = False

    # def goal_response_callback(self, future):
    #     goal_handle = future.result()
    #     if not goal_handle.accepted:
    #         self.get_logger().warn('Goal rejected')
    #         self.move_action_executing_ = False
    #         return

    #     self.get_logger().info('Goal accepted')
    #     result_future = goal_handle.get_result_async()
    #     result_future.add_done_callback(self.goal_result_callback)

    # def goal_result_callback(self, future):
    #     status = future.result().status
    #     if status == GoalStatus.STATUS_SUCCEEDED:
    #         self.get_logger().info("Move completed successfully")
    #     else:
    #         self.get_logger().warn(f"Move failed with status: {status}")
        
    #     self.move_action_executing_ = False

    # def pose_sub_callback(self, msg: PoseStamped):
    #     """Callback when new pose is received"""
    #     self.get_logger().info('Received new pose')

    #     self.get_logger().info("Executing a goal...")
    #     try:

    #         # Get request from the msg
    #         goal_pose = msg
    #         planning_group = 'arm'

    #         # send a request to the IK solver to get joint positions for the desired pose
    #         self.get_logger().info("Finding IK solution")
    #         joint_positions = self.convert_pose_to_joint_angles(goal_pose, planning_group)

    #         if joint_positions is None:
    #             self.get_logger().error("No valid IK solution found, aborting goal")

    #         # create the goal message for the MoveGroup action server
    #         self.get_logger().info("Creating MoveGroup goal message")
    #         move_goal = self.create_move_group_goal(joint_positions, planning_group)

    #         # once the server is up, send the goal
    #         self.move_action_client.wait_for_server()
    #         self.get_logger().info("Sending MoveGroup goal message to MoveIt server")
    #         send_goal_future = self.move_action_client.send_goal_async(move_goal)

    #         # check if the goal was accepted by the MoveGroup action server
    #         if send_goal_future is None:
    #             self.get_logger().error("Failed to send goal to MoveIt action server")

    #         rclpy.spin_until_future_complete(self, send_goal_future)
    #         self.move_action_goal_handle_ = send_goal_future.result()

    #         if not self.move_action_goal_handle_.accepted:
    #             self.get_logger().warn("Goal was rejected by MoveIt action server")

    #         self.get_logger().info("Goal was accepted by MoveIt action server")
    #         get_result_future = self.move_action_goal_handle_.get_result_async()
    #         rclpy.spin_until_future_complete(self, get_result_future)
    #         status = get_result_future.result().status

    #         if status == GoalStatus.STATUS_SUCCEEDED:
    #             self.get_logger().info("MoveIt action server goal successfully executed")
    #         elif status == GoalStatus.STATUS_ABORTED:
    #             self.get_logger().error("MoveIt action server goal aborted")
    #         elif status == GoalStatus.STATUS_CANCELED:
    #             self.get_logger().warn("MoveIt action server goal canceled")

    #     except Exception as e: 
    #         self.get_logger().error(f"Aborting goal\n{e}")

        
    # def move_to_pose_goal_callback(self, goal_request: MoveToPose.Goal):
    #     """
    #     This is the callback for checking if the goal sent to the MoveToPose action server 
    #     was accepted or rejected.

    #     Current policy is: reject new goal if current goal is still executing
    #     """
        
    #     # Policy 1: refuse new goals if current goal is still active
    #     # self.get_logger().info("Checking for active goals...")
    #     # with self.move_to_pose_goal_lock_:
    #     #     if self.move_to_pose_goal_handle_ is not None and self.move_to_pose_goal_handle_.is_active:
    #     #         self.get_logger().info("A goal is already active, rejecting new goal")
    #     #         return GoalResponse.REJECT

    #     self.get_logger().info("Recieved MoveToPose goal request")
    #     return GoalResponse.ACCEPT


    # def move_to_pose_cancel_callback(self, goal_handle: ServerGoalHandle):
    #     self.get_logger().info("Recieved a MoveToPose server cancel request")
    #     return CancelResponse.ACCEPT    # or REJECT
    

    # def move_to_pose_exe_callback(self, goal_handle: ServerGoalHandle):
    #     """
    #     If the goal sent to the MoveToPose action server is accepted then this callback is 
    #     triggered to execute the goal.
    #     """
    #     self.get_logger().info("Executing a goal...")
    #     try:
            
    #         # Policy 1: refuse new goals if current goal is still active
    #         # with self.move_to_pose_goal_lock_:
    #         #     self.move_to_pose_goal_handle_ = goal_handle

    #         # Get request from the goal handle
    #         goal_pose = self.create_goal_pose(goal_handle.request)
    #         planning_group = 'arm'

    #         # send a request to the IK solver to get joint positions for the desired pose
    #         self.get_logger().info("Finding IK solution")
    #         joint_positions = self.convert_pose_to_joint_angles(goal_pose, planning_group)

    #         if joint_positions is None:
    #             self.get_logger().error("No valid IK solution found, aborting goal")
    #             goal_handle.abort()
    #             # with self.move_to_pose_goal_lock_:
    #             #     self.move_to_pose_goal_handle_ = None
    #             result = self.generate_result()
    #             return result

    #         # create the goal message for the MoveGroup action server
    #         self.get_logger().info("Creating MoveGroup goal message")
    #         move_goal = self.create_move_group_goal(joint_positions, planning_group)

    #         # once the server is up, send the goal
    #         self.move_action_client.wait_for_server()
    #         self.get_logger().info("Sending MoveGroup goal message to MoveIt server")
    #         send_goal_future = self.move_action_client.send_goal_async(move_goal)

    #         # check if the goal was accepted by the MoveGroup action server
    #         if send_goal_future is None:
    #             self.get_logger().error("Failed to send goal to MoveIt action server")
    #             goal_handle.abort()
    #             # with self.move_to_pose_goal_lock_:
    #             #     self.move_to_pose_goal_handle_ = None
    #             result = self.generate_result()
    #             return result

    #         rclpy.spin_until_future_complete(self, send_goal_future)
    #         self.move_action_goal_handle_ = send_goal_future.result()

    #         if not self.move_action_goal_handle_.accepted:
    #             self.get_logger().warn("Goal was rejected by MoveIt action server")
    #             goal_handle.abort()
    #             # with self.move_to_pose_goal_lock_:
    #             #     # self.move_to_pose_goal_handle_.abort()
    #             #     self.move_to_pose_goal_handle_ = None
    #             result = self.generate_result()
    #             return result

    #         self.get_logger().info("Goal was accepted by MoveIt action server")
    #         get_result_future = self.move_action_goal_handle_.get_result_async()
    #         rclpy.spin_until_future_complete(self, get_result_future)
    #         status = get_result_future.result().status

    #         if status == GoalStatus.STATUS_SUCCEEDED:
    #             self.get_logger().info("MoveIt action server goal successfully executed")
    #             goal_handle.succeed()
    #             # with self.move_to_pose_goal_lock_:
    #             #     self.move_to_pose_goal_handle_.succeed()
    #         elif status == GoalStatus.STATUS_ABORTED:
    #             self.get_logger().error("MoveIt action server goal aborted")
    #             goal_handle.abort()
    #             # with self.move_to_pose_goal_lock_:
    #             #     self.move_to_pose_goal_handle_.abort()
    #         elif status == GoalStatus.STATUS_CANCELED:
    #             self.get_logger().warn("MoveIt action server goal canceled")
    #             goal_handle.canceled()
    #             # with self.move_to_pose_goal_lock_:
    #             #     self.move_to_pose_goal_handle_.canceled()

    #         # with self.move_to_pose_goal_lock_:
    #         #     # goal_handle = self.move_to_pose_goal_handle_
    #         #     self.move_to_pose_goal_handle_ = None


    #     except Exception as e: 
    #         self.get_logger().error(f"Aborting goal\n{e}")
    #         goal_handle.abort()
    #         # with self.move_to_pose_goal_lock_:
    #         #     self.move_to_pose_goal_handle_ = None

    #     # send the results
    #     result = self.generate_result()

    #     return result        

    ### CLIENT CALLBACKS ###

    # def cancel_move_action_goal(self):
    #     # forward the cancel request to the moveit action server
    #     self.get_logger().info("Forwarding a cancel request to the MoveIt action server")
    #     self.move_action_goal_handle_.cancel_goal_async()

    """
    # def move_action_client_feedback_callback(self, future):
    #     # Handle feedback from action server
    #     pass


    # def move_action_client_response_callback(self, future):
    #     # Handle response from action server
    #     self.move_action_goal_handle_ = future.result()

    #     if self.move_action_goal_handle_.accepted:
    #         # print that the goal was accepted
    #         self.get_logger().info("Goal was accepted by MoveIt action server")
            
    #         self.move_action_goal_handle_.get_result_async().\
    #             add_done_callback(self.move_action_client_result_callback)            

    #     else:
    #         # print that the goal was canceled
    #         self.get_logger().warn("Goal was rejected by MoveIt action server")
            
    #         # set the flag for exiting the while loop in the exe callback
    #         self.move_action_executing_ = False

    #         # set the goal state for the MoveToPose goal handle to aborted
    #         with self.move_to_pose_goal_lock_:
    #             self.move_to_pose_goal_handle_.abort()
        

    # def move_action_client_result_callback(self, future):
    #     # result = future.result().result
    #     status = future.result().status

    #     # set this flag so that we exit the while loop
    #     self.move_action_executing_ = False

    #     # if the goal was successful then set the MoveToPose goal handle to successful as well
    #     if status == GoalStatus.STATUS_SUCCEEDED:
    #         self.get_logger().info("MoveIt action server goal successfully executed")
    #         with self.move_to_pose_goal_lock_:
    #             self.move_to_pose_goal_handle_.succeed()

    #     # if the goal was aborted then set the MoveToPose goal handle to aborted as well
    #     elif status == GoalStatus.STATUS_ABORTED:
    #         self.get_logger().error("MoveIt action server goal aborted")
    #         with self.move_to_pose_goal_lock_:
    #             self.move_to_pose_goal_handle_.abort()

    #     # if the goal was canceled then set the MoveToPose goal handle to canceled as well
    #     elif status == GoalStatus.STATUS_CANCELED:
    #         self.get_logger().warn("MoveIt action server goal canceled")
    #         with self.move_to_pose_goal_lock_:
    #             self.move_to_pose_goal_handle_.canceled()
    """

    ### PUB/SUB CALLBACKS ###
    def joint_state_listener_callback(self, msg):
        # extract the joint state message from the topic
        self.medbot_joint_state = msg

        # if the flag is set to echo topics then echo using the logger
        if self._sub_echo:
            self.get_logger().info('Received joint state:') 
            self.get_logger().info(f' Names: {self.medbot_joint_state.name}') 
            self.get_logger().info(f' Positions: {self.medbot_joint_state.position}') 
            self.get_logger().info(f' Velocities: {self.medbot_joint_state.velocity}') 
            self.get_logger().info(f' Efforts: {self.medbot_joint_state.effort}')


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


    ################# DEFINE HELPER FUNCTIONS #################
    # def create_goal_pose(self, goal_request: MoveToPose.Goal):
    #     """
    #     Takes a goal request and creates a Pose message to be used in the MoveGroup goal.
    #     """
    #     quaternion = self.euler_to_quaternion(goal_request.roll, goal_request.pitch, goal_request.yaw)

    #     goal_pose = Pose()
    #     goal_pose.position.x = goal_request.x
    #     goal_pose.position.y = goal_request.y
    #     goal_pose.position.z = goal_request.z
    #     goal_pose.orientation.x = quaternion[0]
    #     goal_pose.orientation.y = quaternion[1]
    #     goal_pose.orientation.z = quaternion[2]
    #     goal_pose.orientation.w = quaternion[3]

    #     return goal_pose


    # def generate_result(self):
    #     result = MoveToPose.Result()
    #     position, rpy = self.get_end_effector_pose()
    #     result.x = position.x
    #     result.y = position.y
    #     result.z = position.z
    #     result.roll = rpy.x
    #     result.pitch = rpy.y
    #     result.yaw = rpy.z

    #     return result


    async def convert_pose_to_joint_angles(self, goal_pose: PoseStamped, planning_group='arm'):
        """
        Takes a goal pose and makes a call to the IK solver server to find a set of joint 
        angles that satisfy that pose or return nothing if there is no valid solution.
        """
        # Create a request for the GetPositionIK service
        request = GetPositionIK.Request()

        # Populate the request
        request.ik_request.group_name = planning_group
        request.ik_request.robot_state.is_diff = False
        request.ik_request.pose_stamped = goal_pose
        request.ik_request.timeout.sec = 2
        
        try:
            response = await self.ik_service_client.call_async(request)
            if response.error_code.val == response.error_code.SUCCESS:
                return response.solution.joint_state.position
        except Exception as e:
            self.get_logger().error(f"IK failed: {e}")
        return None
        # # Call the service and wait for the response
        # future = self.ik_service_client.call_async(request)
        # rclpy.spin_until_future_complete(self, future, timeout_sec=1.0)
        # response = future.result()

        # if response.error_code.val == response.error_code.SUCCESS:
        #     joint_values = response.solution.joint_state.position
        #     self.get_logger().info(f'Successfully found IK solution: {joint_values}')
        #     return joint_values
        # else:
        #     self.get_logger().error('Failed to find IK solution')
        #     return None


    def create_move_group_goal(self, joint_positions, planning_group='arm'):
        goal = MoveGroup.Goal()

        # Set workspace parameters
        goal = self.set_ws_params(goal)

        # Set start state
        goal = self.set_start_state(goal)
        goal.request.start_state.attached_collision_objects.append(self.create_collision_object())

        # Set goal constraints
        if planning_group=='arm':
            goal_constraints = self.create_joint_angle_constraints(self.arm_joint_names, joint_positions)
        else:
            self.get_logger().error("Invalid planning group specified during goal creation!")

        goal.request.goal_constraints.append(goal_constraints)

        # Set path constraints (empty in this case)
        goal.request.path_constraints = Constraints()

        # Set trajectory constraints (empty in this case)
        goal.request.trajectory_constraints.constraints = []

        # Set other parameters
        goal.request.pipeline_id = 'ompl'
        goal.request.planner_id = ''
        goal.request.group_name = planning_group
        goal.request.num_planning_attempts = 10
        goal.request.allowed_planning_time = 5.0
        goal.request.max_velocity_scaling_factor = 0.1
        goal.request.max_acceleration_scaling_factor = 0.1
        goal.request.cartesian_speed_end_effector_link = ''
        goal.request.max_cartesian_speed = 0.0

        return goal

       
    def create_joint_angle_constraints(self, joint_names, positions):
        constraints = Constraints()

        for i, joint_name in enumerate(joint_names):
            joint_constraint = JointConstraint(
                joint_name=joint_name,
                position=positions[i],
                tolerance_above=0.0001,
                tolerance_below=0.0001,
                weight=1.0
            )
            constraints.joint_constraints.append(joint_constraint)

        return constraints


    def set_ws_params(self, goal:MoveGroup.Goal):
        """
        workspace_parameters=moveit_msgs.msg.WorkspaceParameters(
            header=std_msgs.msg.Header(
                stamp=builtin_interfaces.msg.Time(sec=0, nanosec=0), 
                frame_id=''
            ), 
            min_corner=geometry_msgs.msg.Vector3(x=0.0, y=0.0, z=0.0), 
            max_corner=geometry_msgs.msg.Vector3(x=0.0, y=0.0, z=0.0)
        )

        The workspace_parameters field defines the workspace boundaries for the motion planning process. This 
        helps MoveIt know the spatial limits within which the robot is allowed to move. The components are:

        1. Header:  
            - Definition and Usage: Ensures that the workspace boundaries are defined relative to a specific frame of reference, 
                                    which is critical for accurate motion planning. This field provides metadata about the message.
            - Fields:
                - stamp:        A timestamp indicating when the message was created.
                - frame_id:     The reference frame for the workspace boundaries. Typically, this is set to 
                                a frame such as "world" or "base_link" which the rest of the robot's coordinates 
                                will reference.

        2. Min Corner:  
            - Definition and Usage: Defines the lower bounds of the workspace in 3D space, ensuring that the robot stays within these limits during 
                                    its operations. Specifies the minimum corner of the workspace boundary as a geometry_msgs.msg.Vector3 object.
            - Fields:
                - x: The minimum x-coordinate of the workspace.
                - y: The minimum y-coordinate of the workspace.
                - z: The minimum z-coordinate of the workspace.

        3. Max Corner:  
            - Definition and Usage: Defines the upper bounds of the workspace in 3D space, ensuring that the robot stays within these limits during 
                                    its operations. Specifies the maximum corner of the workspace boundary as a geometry_msgs.msg.Vector3 object.
            - Fields:
                - x: The maximum x-coordinate of the workspace.
                - y: The maximum y-coordinate of the workspace.
                - z: The maximum z-coordinate of the workspace.

        Summary:
        - The workspace_parameters define the spatial boundaries within which the robot is allowed to operate.
        - These boundaries are defined by a minimum and a maximum corner in 3D space.
        - The header ensures that these boundaries are relative to a specific reference frame, which is essential for accurate motion planning.
        - By setting these parameters, you ensure that the robot's movements are confined to a specified area, which is useful for avoiding obstacles 
            and ensuring the robot operates within a safe and designated space. 
        """ 
        
        goal.request.workspace_parameters.header = Header()
        goal.request.workspace_parameters.header.frame_id = 'world'
        goal.request.workspace_parameters.min_corner = self.ws_bounds[0]
        goal.request.workspace_parameters.max_corner = self.ws_bounds[1]

        return goal


    def set_start_state(self, goal:MoveGroup.Goal):
        """
        start_state=moveit_msgs.msg.RobotState(
            joint_state=sensor_msgs.msg.JointState(
                header=std_msgs.msg.Header(stamp=builtin_interfaces.msg.Time(sec=0, nanosec=0), frame_id=''), 
                name=[], 
                position=[], 
                velocity=[], 
                effort=[]
            ),
            multi_dof_joint_state=sensor_msgs.msg.MultiDOFJointState(
                header=std_msgs.msg.Header(stamp=builtin_interfaces.msg.Time(sec=0, nanosec=0), frame_id=''), 
                joint_names=[], 
                transforms=[], 
                twist=[], 
                wrench=[]
            ), 
            attached_collision_objects=[], 
            is_diff=False
        )

        The start_state field specifies the initial state of the robot, including joint states and multi-DOF joint states. This 
        information is used as the starting point for motion planning. The components are:

        1. Joint State:
          - Definition and Usage:   Defines the initial positions, velocities, and efforts for the 1-DOF joints of the robot. This is
                                    essential for starting the motion planning from the current state of the robot. Represents the 
                                    state of the robot's single-degree-of-freedom (1-DOF) joints.
            - Fields:
                - header:   Contains metadata including the timestamp and reference frame.
                - name:     List of joint names corresponding to the state values.
                - position: List of current positions for each joint.
                - velocity: List of current velocities for each joint.
                - effort:   List of current efforts (torques or forces) for each joint.

        2. Multi-DOF Joint State:
          - Definition and Usage:   Defines the initial transformations, velocities, and forces for the multi-DOF joints of the robot. 
                                    This is important for accurately representing the state of complex joints like those in humanoid 
                                    robots or mobile bases. Represents the state of the robot's multi-degree-of-freedom (multi-DOF) joints.
            - Fields:
                - header: Contains metadata including the timestamp and reference frame.
                - joint_names: List of multi-DOF joint names.
                - transforms: List of transformations (translations and rotations) for each joint.
                - twist: List of current twists (combined linear and angular velocities) for each joint.
                - wrench: List of current wrenches (forces and torques) for each joint.

        3. Attached Collision Objects:
          - Definition and Usage:   A list of objects attached to the robot for collision checking.Ensures that any objects being manipulated 
                                    by the robot are considered in the collision environment, preventing collisions with these objects.
            - Fields:
                - a list of the collision objects of type moveit_msgs.msg.AttachedCollisionObject

        4. Is Diff:
          - Definition and Usage:   A boolean flag indicating whether the state is a difference (delta) from the current state. Used to specify 
                                    if the provided state is an incremental change from the current state or a complete state description.

        Summary:
        - The start_state field in the moveit_msgs.msg.RobotState message provides a comprehensive description of the robot's initial 
            state, including single-DOF and multi-DOF joint states, attached objects, and whether the state is incremental or absolute. 
            This information is critical for initiating motion planning from the robot's current configuration, ensuring accurate and 
            efficient path generation.
        """
        
        try:
            goal.request.start_state.joint_state = self.medbot_joint_state
        except:
            self.get_logger().warn("Joint state published not started, may be unable to complete planning requests.")

        goal.request.start_state.is_diff = False

        return goal


    def create_collision_object(self):
        # Create the AttachedCollisionObject 
        attached_object = AttachedCollisionObject() 
        attached_object.link_name = 'link_ee' 
        
        # The end effector link name 
        attached_object.object.id = 'attached_box' 
        attached_object.object.header.frame_id = 'link_ee' 
        
        # Define the primitive (box) 
        box_primitive = SolidPrimitive() 
        box_primitive.type = SolidPrimitive.BOX 
        box_primitive.dimensions = [0.6, 0.6, 0.45] 
        
        # Define the pose of the box 
        box_pose = Pose() 
        box_pose.position.x = 0.0
        box_pose.position.y = 0.0 
        box_pose.position.z = 0.0 
        box_pose.orientation.w = 1.0 
        
        # Add primitive and pose to the collision object 
        attached_object.object.primitives.append(box_primitive) 
        attached_object.object.primitive_poses.append(box_pose) 
        attached_object.object.operation = CollisionObject.ADD 
        
        # Define the touch links (links that the attached object can touch) 
        attached_object.touch_links = ['link_ee'] 
        return attached_object 
    

    # def euler_to_quaternion(self, roll, pitch, yaw):
    #     """Converts Euler angles (in radians) to a quaternion."""
    #     cy = np.cos(yaw * 0.5)
    #     sy = np.sin(yaw * 0.5)
    #     cp = np.cos(pitch * 0.5)
    #     sp = np.sin(pitch * 0.5)
    #     cr = np.cos(roll * 0.5)
    #     sr = np.sin(roll * 0.5)

    #     w = cr * cp * cy + sr * sp * sy
    #     x = sr * cp * cy - cr * sp * sy
    #     y = cr * sp * cy + sr * cp * sy
    #     z = cr * cp * sy - sr * sp * cy

    #     return [x, y, z, w]


    # def quaternion_to_euler(self, x, y, z, w):
    #     """
    #     Convert a quaternion into euler angles (roll, pitch, yaw)
    #     roll is rotation around x-axis
    #     pitch is rotation around y-axis
    #     yaw is rotation around z-axis
    #     """
    #     # Roll (x-axis rotation)
    #     sinr_cosp = 2 * (w * x + y * z)
    #     cosr_cosp = 1 - 2 * (x * x + y * y)
    #     roll = np.arctan2(sinr_cosp, cosr_cosp)

    #     # Pitch (y-axis rotation)
    #     sinp = 2 * (w * y - z * x)
    #     if np.abs(sinp) >= 1:
    #         pitch = np.sign(sinp) * np.pi / 2  # use 90 degrees if out of range
    #     else:
    #         pitch = np.arcsin(sinp)

    #     # Yaw (z-axis rotation)
    #     siny_cosp = 2 * (w * z + x * y)
    #     cosy_cosp = 1 - 2 * (y * y + z * z)
    #     yaw = np.arctan2(siny_cosp, cosy_cosp)

    #     return roll, pitch, yaw


    # def get_end_effector_pose(self): 
        
    #     pos = Vector3()
    #     rpy = Vector3()
        
    #     try: 
    #         # Lookup transform from base_link to end_effector 
            
    #         self.ee_pose.position
            
    #         # Extract translation and rotation from the transform 
    #         translation = self.ee_pose.position 
    #         rotation = self.ee_pose.orientation

    #         pos.x = translation.x
    #         pos.y = translation.y
    #         pos.z = translation.z

    #         roll, pitch, yaw = self.quaternion_to_euler(rotation.x, rotation.y, rotation.z, rotation.w)

    #         rpy.x = roll
    #         rpy.y = pitch
    #         rpy.z = yaw
            
    #         if self._sub_echo:
    #             self.get_logger().info(f"End Effector Position: x={translation.x}, y={translation.y}, z={translation.z}") 
    #             self.get_logger().info(f"End Effector Orientation: x={rotation.x}, y={rotation.y}, z={rotation.z}, w={rotation.w}") 
            
    #     except Exception as e: 
    #         self.get_logger().warn(f"Could not get transform: {e}")

    #     return pos, rpy


    # ### UNUSED FUNCTIONS (FROM PROTOTYPING, DELETE ONCE NOT NEEDED)
    # def send_goal_to_move_group(self, goal_pose: Pose, planning_group='arm'):
        
    #     try:
    #         # create the goal
    #         goal = self.create_move_group_goal(goal_pose, planning_group)
            
    #         # once the server is up, send the goal
    #         self.move_action_client.wait_for_server()
    #         future = self.move_action_client.send_goal_async(goal, feedback_callback=self.goal_feedback_callback).add_done_callback(self.goal_response_callback)
        
    #     except Exception as e: 
    #         self.get_logger().error(f"Aborting goal\n{e}")
    #         ### ABORT THE GOAL


def main(args=None):
    rclpy.init(args=args)
    node = MoveToPoseNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
