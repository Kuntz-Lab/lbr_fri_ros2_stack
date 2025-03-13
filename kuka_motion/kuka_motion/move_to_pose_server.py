#!/usr/bin/env python3
# filepath: /home/joe/medbot_ws/src/medbot_ros2_stack/wrapper_action_server.py

import rclpy
import threading
from rclpy.node import Node
from rclpy.action import ActionServer, ActionClient, CancelResponse, GoalResponse
from rclpy.action.server import ServerGoalHandle
from rclpy.action.client import GoalStatus
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import Pose, Vector3, PoseStamped
from kuka_interfaces.action import MoveToPose as WrapperAction
from tf2_ros import TransformListener, Buffer
from sensor_msgs.msg import JointState
from moveit_msgs.srv import GetPositionIK
from moveit_msgs.action import MoveGroup as TargetAction
from moveit_msgs.msg import Constraints, JointConstraint, AttachedCollisionObject, CollisionObject
from std_msgs.msg import Header
from shape_msgs.msg import SolidPrimitive

def clamp(value, min_value, max_value):
    """Clamp a value between min and max values."""
    return max(min(value, max_value), min_value)

class MoveToPoseServer(Node):
    def __init__(self):
        super().__init__('move_to_pose_server')

        self._sub_echo = False

        # Set the robot name parameter and assocuated topics/services/actions
        self.declare_parameter('robot_name', 'med14_tc')
        self.robot_name = self.get_parameter('robot_name').value

        self.planning_gropus = ['arm', 'gripper']

        self.joint_state_topic_name = 'lbr/joint_states'
        self.move_action_name = '/lbr/move_action'
        self.joint_names = {
            'arm': ['lbr_A1', 'lbr_A2', 'lbr_A3', 'lbr_A4', 'lbr_A5', 'lbr_A6', 'lbr_A7'],
            'gripper': ['lbr_finger_joint'],
            'all': ['lbr_left_inner_finger_joint',
                      'lbr_right_inner_knuckle_joint',
                      'lbr_right_outer_knuckle_joint',
                      'lbr_right_inner_finger_joint',
                      'lbr_left_inner_knuckle_joint',
                      'lbr_A2',
                      'lbr_A3',
                      'lbr_A4',
                      'lbr_A6',
                      'lbr_finger_joint',
                      'lbr_A1',
                      'lbr_A5',
                      'lbr_A7']
            }
        self.ws_bounds = (Vector3(x=-1.0,y=-1.0,z=-1.0), Vector3(x=1.0,y=1.0,z=1.0))
        self.ik_solver_name = '/lbr/compute_ik'

        if self.robot_name == 'med14':
            self.ee_frame = 'lbr_link_7'
            self.base_frame = 'lbr_link_1'
            self.get_logger().info('M2P SRV: Robot name set to "med14"')

        elif self.robot_name == 'med14_tc':
            self.ee_frame = 'lbr_tendon_robot_link'
            self.base_frame = 'lbr_link_1'
            self.get_logger().info('M2P SRV: Robot name set to "med14_tc"')

        elif self.robot_name == 'med14_robotiq_2f':
            self.ee_frame = 'lbr_robotiq_140_base_link'
            self.base_frame = 'lbr_floating_link'
            self.get_logger().info('M2P SRV: Robot name set to "med14_robotiq_2f"')

        else:
            self.get_logger().warn('M2P SRV: No valid robot name specified in MoveToGoal() initialization, various topics/services/actions may not work!') 
            self.joint_state_topic_name = '/joint_states'
            self.ee_frame = 'lbr_link_7'
            self.base_frame = 'lbr_link_1'

        # Use ReentrantCallbackGroup to allow concurrent callbacks
        self.callback_group = ReentrantCallbackGroup()
        
        # Add thread lock for goal state management
        self._lock = threading.Lock()
        # Flag to track if a goal is active
        self._is_active = False
        # Store the currently active goal handle
        self._current_goal_handle = None

        ############################################
        # IK SERVICE CLIENT
        ############################################
        self.ik_service_client = self.create_client(
            GetPositionIK, 
            self.ik_solver_name,
            callback_group=self.callback_group
            )
                # wait for move_group to be started
        while not self.ik_service_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('M2P SRV: IK service not available, waiting for IK service...')
        self.get_logger().info("M2P SRV: IK service has been started")

        ############################################
        # M2P WRAPPER ACTION SERVER (MOVE_TO_POSE)
        ############################################
        # Create the action server - this is what clients will connect to
        self._action_server = ActionServer(
            self,
            WrapperAction,  # The action type for this wrapper
            'move_to_pose',  # The action name
            self.execute_callback,  # Called when a goal is accepted
            callback_group=self.callback_group,
            goal_callback=self.goal_callback,  # Called when a goal is received
            cancel_callback=self.cancel_callback,  # Called when a cancel request is received
        )
        
        ############################################
        # TARGET ACTION CLIENT (MOVE_ACTION)
        ############################################
        # Create the action client - this connects to the target server (move_action)
        self._action_client = ActionClient(
            self,
            TargetAction,  # The target action type
            self.move_action_name,  # The target action name
            callback_group=self.callback_group
        )
        while not self._action_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().info('M2P SRV: Target action server (move_action) not available, waiting for target action server...')
        self.get_logger().info("M2P SRV: Target action server (move_action) has been started")
        
        ############################################
        # TF BUFFER AND LISTENER
        ############################################
        # Create a TF buffer and listener, objects for storing joint state and ee pose
        self.tf_buffer = Buffer() 
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.ee_pose = Pose()
        self.tf_update_timer = self.create_timer(
            0.002,       # update at 500 hz
            self.ee_pose_listener_callback
            )
        self.get_logger().info("M2P SRV: End-effector pose listener has been started")

        ############################################
        # JOINT STATE SUBSCRIBER
        ############################################
        # create joint state subscriber
        self.current_joint_state = {
            'arm': JointState(),
            'gripper': JointState(),
            'total': JointState()
            }
        self.joint_state_sub_ = self.create_subscription(
            JointState,
            self.joint_state_topic_name,
            self.joint_state_listener_callback,
            100     # the topic publishes at approx. 200 hz so can go up to that if needed
            )
        self.get_logger().info("M2P SRV: MoveToPose joint state subscriber has been started")

        # Dictionary to keep track of active target goal handle
        self._target_goal_handle = None
        
        self.get_logger().info('M2P SRV: Wrapper action server has been started')

    # /***************************************************************************************************/
    # /*    EE_POSE_LISTENER_CALLBACK                                                                    */
    # /***************************************************************************************************/
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
                self.get_logger().info(f"\nM2P SRV: End Effector Position: x={translation.x}, y={translation.y}, z={translation.z}\nEnd Effector Orientation: x={rotation.x}, y={rotation.y}, z={rotation.z}, w={rotation.w}") 
            
        except Exception as e: 
            self.get_logger().warn(f"M2P SRV: Could not get transform: {e}")

    # /***************************************************************************************************/
    # /*    JOINT_STATE_LISTENER_CALLBACK                                                                */
    # /***************************************************************************************************/
    def joint_state_listener_callback(self, msg):
        self.current_joint_state['all'] = msg
        # extract the joint state message from the topic
        for planning_group in self.planning_gropus:
            # add the appropriate joint states to the appropriate entry in the dictionary
            # Create a new joint state message for this planning group
            group_joint_state = JointState()
            group_joint_state.header = msg.header

            # Ensure joints are in the same order as defined in joint_names
            ordered_state = JointState()
            ordered_state.header = group_joint_state.header
            for name in self.joint_names[planning_group]:
                if name in group_joint_state.name:
                    idx = group_joint_state.name.index(name)
                    ordered_state.name.append(name)
                    ordered_state.position.append(group_joint_state.position[idx])
                    # Also handle velocity and effort
            self.current_joint_state[planning_group] = ordered_state
            
            # # Get the joint names for this planning group
            # group_joint_names = self.joint_names[planning_group]
            
            # # For each joint in the incoming message
            # for i, joint_name in enumerate(msg.name):
            #     # If this joint belongs to the current planning group
            #     if joint_name in group_joint_names:
            #         # Add this joint to the group's joint state
            #         group_joint_state.name.append(joint_name)
            #         group_joint_state.position.append(msg.position[i])
                    
            #         if msg.velocity:
            #             group_joint_state.velocity.append(msg.velocity[i])
                    
            #         if msg.effort:
            #             group_joint_state.effort.append(msg.effort[i])
            
            # # Store the filtered joint state in the dictionary
            # self.current_joint_state[planning_group] = group_joint_state

        # if the flag is set to echo topics then echo using the logger
        if self._sub_echo:
            for planning_group in self.planning_gropus:
                self.get_logger().info('Received joint state:') 
                self.get_logger().info(f' Names: {self.current_joint_state[planning_group].name}') 
                self.get_logger().info(f' Positions: {self.current_joint_state[planning_group].position}') 
                self.get_logger().info(f' Velocities: {self.current_joint_state[planning_group].velocity}') 
                self.get_logger().info(f' Efforts: {self.current_joint_state[planning_group].effort}')

    # /***************************************************************************************************/
    # /*    M2P GOAL_CALLBACK                                                                            */
    # /***************************************************************************************************/
    def goal_callback(self, goal_request):
        """Called when a goal is received from a client"""
        self.get_logger().info('M2P SRV: Received goal request')
        
        # Use thread locking to check if a goal is already active
        with self._lock:
            if self._is_active:
                self.get_logger().warn('M2P SRV: Rejecting goal: Another goal is already active')
                return GoalResponse.REJECT
            else:
                # No active goal, check if there is a valid IK solution
                try:
                    planning_group = goal_request.planning_group
                except:
                    planning_group = 'arm'
                response = self.get_ik_solution(goal_request.desired_pose, planning_group)
                if response is None:
                    self.get_logger().warn('M2P SRV: Rejecting goal: No IK solution found')
                    return GoalResponse.REJECT
                else:
                    self.get_logger().info('M2P SRV: Accepting goal')
                    return GoalResponse.ACCEPT

    # /***************************************************************************************************/
    # /*    M2P CANCEL_CALLBACK                                                                          */
    # /***************************************************************************************************/
    def cancel_callback(self, goal_handle):
        """Called when a cancel request is received"""
        self.get_logger().info('M2P SRV: Received cancel request')
        
        with self._lock:
            # Only allow cancellation of the current active goal
            if goal_handle == self._current_goal_handle:
                # Cancel the target goal if it exists
                if self._target_goal_handle:
                    self.get_logger().info('M2P SRV: Cancelling target goal')
                    self._target_goal_handle.cancel_goal_async(self._target_goal_handle)
                    
                return CancelResponse.ACCEPT
            
        return CancelResponse.REJECT

    # /***************************************************************************************************/
    # /*    M2P EXECUTE_CALLBACK                                                                         */
    # /***************************************************************************************************/
    async def execute_callback(self, goal_handle):
        """Called when a goal is accepted and should be executed"""
        self.get_logger().info('M2P SRV: Executing goal...')
        
        # Set goal as active with thread safety
        with self._lock:
            self._is_active = True
            self._current_goal_handle = goal_handle
        
        # Create a result object for possible early returns
        wrapper_result = WrapperAction.Result()
        
        try:
            # extract the goal pose from the goal handle
            wrapper_goal_pose = goal_handle.request.desired_pose
            # get planning group
            try:
                planning_group = goal_handle.request.planning_group
            except:
                planning_group = 'arm'

            # Get IK solution for the goal pose
            ik_response = await self.get_ik_solution(wrapper_goal_pose)
            if ik_response is None:
                self.get_logger().warn('M2P SRV: No IK solution found')
                goal_handle.abort()
                wrapper_result.message = 'No IK solution found'
                wrapper_result.success = False
                return wrapper_result
            
            # Wait for the target action server to be available
            if not self._action_client.wait_for_server(timeout_sec=1.0):
                self.get_logger().error('M2P SRV: Target action server not available')
                goal_handle.abort()
                wrapper_result.message = 'Target action server not available'
                wrapper_result.success = False
                return wrapper_result
            
            # get velocity scaling
            try:
                vel_scaling = goal_handle.request.vel_scaling
                vel_scaling = clamp(vel_scaling, 0.0, 1.0)
            except:
                vel_scaling = 0.1

            # get acceleration scaling
            try:
                acc_scaling = goal_handle.request.acc_scaling
                acc_scaling = clamp(acc_scaling, 0.0, 1.0)
            except:
                acc_scaling = 0.1

            # Map ik_response and planning params to target_goal
            target_goal = self.create_move_group_goal(ik_response, planning_group, vel_scaling, acc_scaling)
            
            # Send the goal to the target action server
            send_goal_future = self._action_client.send_goal_async(
                target_goal,
                feedback_callback=lambda feedback_msg: self._feedback_callback(goal_handle, feedback_msg)
            )
            
            # Wait for the target server to accept the goal
            target_goal_handle = await send_goal_future
            if not target_goal_handle.accepted:
                self.get_logger().error('M2P SRV: Target server rejected the goal')
                goal_handle.abort()
                wrapper_result.message = 'Target server rejected the goal'
                wrapper_result.success = False
                return wrapper_result
            
            # Store the target goal handle (thread-safe)
            with self._lock:
                self._target_goal_handle = target_goal_handle
            
            # Request the result from the target server
            target_result_future = target_goal_handle.get_result_async()
            
            # Wait for the target action to complete
            target_result = await target_result_future
            
            # Check if our wrapper goal was cancelled while waiting for the target
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                wrapper_result.message = 'Goal was cancelled'
                wrapper_result.success = False
                return wrapper_result
            
            # Create a PoseStamped object for the result
            reached_pose_stamped = PoseStamped()
            reached_pose_stamped.header.stamp = self.get_clock().now().to_msg()
            reached_pose_stamped.header.frame_id = self.base_frame
            reached_pose_stamped.pose = self.ee_pose

            # Map target_result to wrapper_result
            wrapper_result.message = "Target action completed, robot done moving"
            wrapper_result.reached_pose = reached_pose_stamped
            wrapper_result.success = target_result.status == GoalStatus.STATUS_SUCCEEDED
            
            # Determine final status based on target result status
            if target_result.status == GoalStatus.STATUS_SUCCEEDED:
                self.get_logger().info('M2P SRV: Target action completed successfully')
                goal_handle.succeed()
            else:
                self.get_logger().warn(f'M2P SRV: Target action failed with status: {target_result.status}')
                goal_handle.abort()
                
            return wrapper_result
        
        except Exception as e:
            self.get_logger().error(f'M2P SRV: Failed to execute goal: {e}')
            goal_handle.abort()
            wrapper_result.message = 'Failed to execute goal'
            wrapper_result.success = False
            return wrapper_result
            
        finally:
            # Always clean up state when done (thread-safe)
            with self._lock:
                self._is_active = False
                self._current_goal_handle = None
                self._target_goal_handle = None

    # /***************************************************************************************************/
    # /*    M2P FEEDBACK_CALLBACK                                                                        */
    # /***************************************************************************************************/
    def _feedback_callback(self, goal_handle, feedback_msg):
        """
        Called when feedback is received from the target action server

        From MoveToPose.action:
        # Feedback: current pose and planning state
        geometry_msgs/PoseStamped current_pose
        string state
        """

        # Convert target feedback to wrapper feedback
        wrapper_feedback = WrapperAction.Feedback()
        # Map target_feedback to wrapper_feedback
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = self.base_frame
        wrapper_feedback.current_pose.header = header
        wrapper_feedback.current_pose.pose = self.ee_pose   
        wrapper_feedback.state = "Executing motion plan"

        # Publish feedback to the wrapper client
        goal_handle.publish_feedback(wrapper_feedback)

    # /***************************************************************************************************/
    # /*    GET_IK_SOLUTION                                                                              */
    # /***************************************************************************************************/
    async def get_ik_solution(self, pose_msg, planning_group='arm'):
        """Non-blocking IK service call"""
        request = GetPositionIK.Request()
        request.ik_request.pose_stamped = pose_msg
        request.ik_request.group_name = planning_group
        request.ik_request.ik_link_name = self.ee_frame
        # request.ik_request.robot_state = self.medbot_joint_state
        request.ik_request.timeout = rclpy.time.Duration(seconds=1.0).to_msg()

        try:
            response = await self.ik_service_client.call_async(request)
            if response.error_code.val == response.error_code.SUCCESS:
                return response.solution.joint_state.position
            else:
                self.get_logger().warn(f"IK failed: {response.error_code}")
        except Exception as e:
            self.get_logger().error(f"IK failed: {e}")
        return None
    
    # /***************************************************************************************************/
    # /*    CREATE_MOVE_GROUP_GOAL                                                                       */
    # /***************************************************************************************************/
    def create_move_group_goal(self, joint_positions, planning_group='arm', vel_scaling=0.1, acc_scaling=0.1):
        try:
            goal = TargetAction.Goal()

            # Set workspace parameters
            goal = self.set_ws_params(goal)

            # Set start state
            goal = self.set_start_state(goal)

            # attach collision objects
            # goal.request.start_state.attached_collision_objects.append(self.create_collision_object())

            # Set goal constraints
            try:
                goal_constraints = self.create_joint_angle_constraints(self.joint_names[planning_group], joint_positions)
            except:
                self.get_logger().error("Invalid planning group specified during goal creation! Possible mismatch between joint positions found by IK solver and joint names specified!")
                return None
            
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
            goal.request.max_velocity_scaling_factor = vel_scaling
            goal.request.max_acceleration_scaling_factor = acc_scaling
            goal.request.cartesian_speed_end_effector_link = ''
            goal.request.max_cartesian_speed = 0.0

            return goal

        except Exception as e:
            self.get_logger().error(f"Error creating move group goal: {e}")
            return None

    # /***************************************************************************************************/
    # /*    CREATE_JOINT_ANGLE_CONSTRAINTS                                                               */
    # /***************************************************************************************************/
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

    # /***************************************************************************************************/
    # /*    SET_WS_PARAMS                                                                                */
    # /***************************************************************************************************/
    def set_ws_params(self, goal:TargetAction.Goal):
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

    # /***************************************************************************************************/
    # /*    SET_START_STATE                                                                              */
    # /***************************************************************************************************/
    def set_start_state(self, goal:TargetAction.Goal):
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
            goal.request.start_state.joint_state = self.current_joint_state['arm']
        except:
            self.get_logger().warn("Joint state published not started, may be unable to complete planning requests.")

        goal.request.start_state.is_diff = False

        return goal

    # /***************************************************************************************************/
    # /*    CREATE_COLLISION_OBJECT                                                                      */
    # /***************************************************************************************************/
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
    


def main(args=None):
    rclpy.init(args=args)
    
    # Create node
    m2p_action_server = MoveToPoseServer()
    
    # Use MultiThreadedExecutor to allow concurrent callbacks
    executor = MultiThreadedExecutor()
    executor.add_node(m2p_action_server)
    
    try:
        # Start processing callbacks
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        # Clean up
        executor.shutdown()
        m2p_action_server.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()