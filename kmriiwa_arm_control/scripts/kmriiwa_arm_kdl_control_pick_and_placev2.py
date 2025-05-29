#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from action_msgs.msg import GoalStatus
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from sensor_msgs.msg import JointState
from numpy import deg2rad
import numpy as np
import PyKDL
from kdl_parser_py import urdf
from urdf_parser_py.urdf import URDF
from threading import Lock
import time

class KDLActionClient(Node):
    def __init__(self):
        super().__init__('kdl_action_client')
        
        # Initialize KDL-related members
        self.joint_states_arr = np.zeros(7)
        self.mutex = Lock()
        self.setup_kdl()
        
        # Setup action client
        self.callback_group = ReentrantCallbackGroup()
        self._action_client = ActionClient(
            self,
            FollowJointTrajectory,
            '/kmriiwa/arm_controller/follow_joint_trajectory',
            callback_group=self.callback_group
        )
        
        # Joint state subscriber
        self.joint_state_subscriber = self.create_subscription(
            JointState,
            'kmriiwa/arm/joint_states',
            self.joint_state_callback,
            1
        )
        
        # Wait for action server
        self.get_logger().info('Waiting for action server...')
        if not self._action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Action server not available')
            raise RuntimeError('Action server not available')
        self.get_logger().info('Action server found!')

        self.joint_names = [
            'kmriiwa_joint_1',
            'kmriiwa_joint_2',
            'kmriiwa_joint_3',
            'kmriiwa_joint_4',
            'kmriiwa_joint_5',
            'kmriiwa_joint_6',
            'kmriiwa_joint_7'
        ]

    def setup_kdl(self):
        """Setup KDL chain from URDF."""
        try:
            # Load URDF and build KDL tree
            build_ok, kdl_tree = urdf.treeFromFile('/home/imr/ros2_ws_coresense/src/kmriiwa_ros_stack/kmriiwa_description/urdf/robot/kmriiwakdl.urdf')
            if not build_ok:
                raise RuntimeError('Failed to build KDL tree')
            
            # Setup KDL chain
            self.base_link = "kmriiwa_link_0"
            self.ee_link = "kmriiwa_link_ee"
            self.kdl_chain = kdl_tree.getChain(self.base_link, self.ee_link)
            
            # Get number of joints
            self.no_of_joints = self.kdl_chain.getNrOfJoints()
            
            # Setup KDL solvers
            self.fk_solver = PyKDL.ChainFkSolverPos_recursive(self.kdl_chain)
            self.ik_solver_vel = PyKDL.ChainIkSolverVel_pinv(self.kdl_chain)
            self.ik_solver = PyKDL.ChainIkSolverPos_NR(
                self.kdl_chain,
                self.fk_solver,
                self.ik_solver_vel,
                100,  # Max iterations
                1e-6  # Epsilon
            )
            
            self.get_logger().info('KDL setup completed successfully')
            
        except Exception as e:
            self.get_logger().error(f'Failed to setup KDL: {str(e)}')
            raise

    def joint_state_callback(self, joint_states):
        """Store current joint states."""
        with self.mutex:
            for i in range(self.no_of_joints):
                self.joint_states_arr[i] = joint_states.position[i]

    def create_goal_msg(self, positions, duration=5.0):
        """Create a FollowJointTrajectory goal message."""
        trajectory = JointTrajectory()
        trajectory.joint_names = self.joint_names
        
        point = JointTrajectoryPoint()
        point.positions = positions
        point.time_from_start = Duration(sec=int(duration), nanosec=int((duration % 1) * 1e9))
        
        trajectory.points = [point]
        
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory = trajectory
        
        return goal_msg

    def move_to_joint_pose(self, positions):
        """Send a joint position goal and wait for result."""
        goal_msg = self.create_goal_msg(positions)
        
        self.get_logger().info('Sending goal...')
        send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        
        # Wait for goal acceptance
        rclpy.spin_until_future_complete(self, send_goal_future)
        goal_handle = send_goal_future.result()
        
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected')
            return False
            
        self.get_logger().info('Goal accepted')
        
        # Wait for result
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        
        status = result_future.result().status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('Goal succeeded!')
            return True
        else:
            self.get_logger().error(f'Goal failed with status: {status}')
            return False

    def feedback_callback(self, feedback_msg):
        """Handle action feedback."""
        feedback = feedback_msg.feedback
        # Process feedback if needed
        pass

    def get_current_tcp_pose(self):
        """Get current TCP pose using forward kinematics."""
        with self.mutex:
            if np.all(self.joint_states_arr == 0):
                self.get_logger().error("No joint states received")
                return None
                
            try:
                # Create joint array with current positions
                current_joints = PyKDL.JntArray(self.no_of_joints)
                for i in range(self.no_of_joints):
                    current_joints[i] = self.joint_states_arr[i]
                
                # Calculate forward kinematics
                current_frame = PyKDL.Frame()
                if self.fk_solver.JntToCart(current_joints, current_frame) < 0:
                    self.get_logger().error('Failed to calculate forward kinematics')
                    return None
                
                # Convert KDL frame to transformation matrix
                rotation = current_frame.M
                position = current_frame.p
                
                return current_frame
                    
            except Exception as e:
                self.get_logger().error(f'Error getting current TCP pose: {str(e)}')
                return None

    def joint_pick_and_place(self):
        """Execute pick and place sequence using joint positions."""
        # Define joint positions sequence
        start_approach_pick = deg2rad([84.42, -41.77, 0.0, 90.21, 0.0, -48.04, 54.44]).tolist()
        start_pick_points = deg2rad([84.42, -48.38, 0.0, 90.47, 0.0, -41.14, 54.44]).tolist()
        clearing_pose_1 = deg2rad([84.42, -30.07, 0.0, 36.38, 0.0, -113.55, 54.44]).tolist()
        clearing_pose_2 = deg2rad([-9.06, -1.10, 0.0, 73.92, 0.0, -104.98, 51.55]).tolist()
        clearing_pose_3 = deg2rad([-90.85, -1.10, 0.0, 73.92, 0.0, -104.98, 51.55]).tolist()
        scan_aruco_pose = deg2rad([-90.12, -43.15, 0.0, 40.52, 0.0, -96.34, 52.27]).tolist()

        # Execute sequence
        sequence = [
            ('Approach pick', start_approach_pick),
            ('Pick position', start_pick_points),
            ('Clearing pose 1', clearing_pose_1),
            ('Clearing pose 2', clearing_pose_2),
            ('Clearing pose 3', clearing_pose_3),
            ('Scan aruco pose', scan_aruco_pose)
        ]

        # Wait for initial joint states
        start_time = time.time()
        while time.time() - start_time < 3.0:  # Wait up to 3 seconds
            rclpy.spin_once(self, timeout_sec=0.1)
            if not np.all(self.joint_states_arr == 0):
                break

        if np.all(self.joint_states_arr == 0):
            self.get_logger().error("No joint states received!")
            return False

        for name, positions in sequence:
            self.get_logger().info(f'Moving to {name}...')
            if not self.move_to_joint_pose(positions):
                self.get_logger().error(f'Failed at {name}')
                return False
            time.sleep(2.0)  # Wait between movements

        self.get_logger().info('Pick and place sequence completed successfully')
        return True

def main(args=None):
    rclpy.init(args=args)
    action_client = KDLActionClient()
    
    try:
        result = action_client.joint_pick_and_place()
        if result:
            action_client.get_logger().info('Pick and place completed successfully')
        else:
            action_client.get_logger().error('Pick and place failed')
    except KeyboardInterrupt:
        action_client.get_logger().info('Interrupted by user')
    except Exception as e:
        action_client.get_logger().error(f'Error occurred: {str(e)}')
    finally:
        action_client.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()