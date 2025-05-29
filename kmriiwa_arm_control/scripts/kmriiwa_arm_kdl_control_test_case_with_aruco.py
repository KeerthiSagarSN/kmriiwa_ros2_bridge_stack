#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from kmriiwa_msgs.msg import JointPosition
import numpy as np
from threading import Lock
import PyKDL
#from kdl_parser_py.kdl_parser_py import kdl_tree_from_urdf_model
from urdf_parser_py.urdf import URDF
from numpy import deg2rad,rad2deg,array,sign

# from kdl_parser_py import KDL
from kdl_parser_py import urdf
import tf2_ros
from geometry_msgs.msg import PoseStamped, TransformStamped
from tf2_geometry_msgs import do_transform_pose
import time
from copy import deepcopy
import math

from visualization_msgs.msg import MarkerArray
from collections import defaultdict
from scipy.spatial.transform import Rotation
import threading
from rclpy.duration import Duration
############# Only for Testing ######################################################
class ArmManipulationClient(Node):
    def __init__(self):
        super().__init__('arm_manipulation_client')
        
        # Original members
        self.joint_states_positions = JointState()
        self.joint_states_arr = np.zeros(7) ## Numpy array for calaculation throurhgt
        #self.joints_states_arr = np.array([1.50,-0.707,0.0,1.570,0,-0.707,0])
        #self.joint_states_arr = None

        ## All threading shit here
        self.mutex1 = Lock()
        self.mutex2 = Lock()
        self.traj_response = String()
        self.trajectory_completed_event = threading.Event()
        self.trajectory_result = False
        
        # KDL related members
        self.robot_urdf = None
        self.kdl_chain = None
        self.fk_solver = None
        self.ik_solver = None
        self.ik_solver_vel = None

        self.no_of_joints = 0
        
        
        ## For aruco markers
        # Dictionary to store latest poses for each marker ID
        self.marker_poses = defaultdict(dict)
        
        # Original subscribers and publishers
        self.joint_state_subscriber = self.create_subscription(
            JointState,
            'kmriiwa/arm/joint_states',
            self.joint_state_callback,
            1
        )
        
        self.response_trajectory = self.create_subscription(
            String,
            'kmriiwa/arm/state/JointPositionReached',
            self.trajectory_action_completed,
            10
        )
        
        self.gripper_close_publisher = self.create_publisher(
            String,
            'kmriiwa/base/command/gripperActionClose',
            1
        )
        
        self.gripper_open_publisher = self.create_publisher(
            String,
            'kmriiwa/base/command/gripperActionOpen',
            1
        )
        
        self.traj_desired_publisher = self.create_publisher(
            JointPosition,
            'kmriiwa/arm/command/JointPosition',
            1
        )
        

        self.aruco_markers_subscriber = self.create_subscription(MarkerArray,'/coresense/assembly_station/aruco_markers',self.aruco_marker_callback,1)
        # Setup KDL
        self.setup_kdl()
        # TF2 buffer for transforms
        # self.tf_buffer = tf2_ros.Buffer()
        # self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

    def setup_kdl(self):
        """Setup KDL chain from URDF."""
        try:
           
            
            # Adapting filestructure from my old code with Telerobot

            robot_description = URDF.from_xml_file('/home/imr/ros2_ws_coresense/src/kmriiwa_ros_stack/kmriiwa_description/urdf/robot/kmriiwakdl.urdf')

            # Build the tree here from the URDF parser file from the file location

            build_ok, kdl_tree = urdf.treeFromFile('/home/imr/ros2_ws_coresense/src/kmriiwa_ros_stack/kmriiwa_description/urdf/robot/kmriiwakdl.urdf')

            if build_ok == True:
                print('KDL chain built successfully !!')
            else:
                print('KDL chain unsuccessful')


            
            # Create KDL tree
            self.base_link = "kmriiwa_link_0"
            #self.ee_link = "kmriiwa_link_ee"
            ## I have gripper stl now- keeping itmore realistic for simulation testing
            self.ee_link = "kmriiwa_gripper_tcp"

            # Build the kdl_chain here
            self.kdl_chain = kdl_tree.getChain(self.base_link, self.ee_link)
            
            

            self.no_of_joints = self.kdl_chain.getNrOfJoints()
            print('No of joints are',self.no_of_joints)
            print('Current joint states are',self.joint_states_arr)

            
            ### All declarative variables for Kinematics are defined here ###################################
            # Setting up input joints array form joint_states_callback here
            self.joint_states_kdl = PyKDL.JntArray(self.no_of_joints)

            

            # All solvers, fk, ik_vel for cartesina twist and ik_pos
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
    
    
    def quaternion_to_euler(self,quaternion):
        """
        Convert quaternion to Euler angles (roll, pitch, yaw).
        
        Args:
            quaternion: Quaternion as [x, y, z, w]
            
        Returns:
            Euler angles as (roll, pitch, yaw) in radians
        """
        x, y, z, w = quaternion
        
        # Roll (rotation around x-axis)
        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = math.atan2(sinr_cosp, cosr_cosp)
        
        # Pitch (rotation around y-axis)
        sinp = 2 * (w * y - z * x)
        if abs(sinp) >= 1:
            # Use 90 degrees if out of range
            pitch = math.copysign(math.pi / 2, sinp)
        else:
            pitch = math.asin(sinp)
        
        # Yaw (rotation around z-axis)
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        
        return (roll, pitch, yaw)
    
    def euler_to_quaternion(self,roll, pitch, yaw):
        """
        Convert Euler angles to quaternion.
        
        Args:
            roll: Rotation around x-axis in radians
            pitch: Rotation around y-axis in radians
            yaw: Rotation around z-axis in radians
            
        Returns:
            Quaternion as [x, y, z, w]
        """
        # Abbreviations for the various angular functions
        cy = np.cos(yaw * 0.5)
        sy = np.sin(yaw * 0.5)
        cp = np.cos(pitch * 0.5)
        sp = np.sin(pitch * 0.5)
        cr = np.cos(roll * 0.5)
        sr = np.sin(roll * 0.5)
        
        # Quaternion
        x = sr * cp * cy - cr * sp * sy
        y = cr * sp * cy + sr * cp * sy
        z = cr * cp * sy - sr * sp * cy
        w = cr * cp * cy + sr * sp * sy
        
        return np.array([x, y, z, w])

    def aruco_marker_callback(self, msg):
        # Clear old poses
        self.marker_poses.clear()
        
        for marker in msg.markers:
            marker_id = marker.id
            pos = marker.pose.position
            ori = marker.pose.orientation
            
            # Store pose for this ID
            self.marker_poses[marker_id] = {
                'position': {
                    'x': pos.x,
                    'y': pos.y,
                    'z': pos.z
                },
                'orientation': {
                    'x': ori.x,
                    'y': ori.y,
                    'z': ori.z,
                    'w': ori.w
                }
            }
            
            # Log pose for this ID
            # self.get_logger().info(
            #     f'\nMarker ID {marker_id}:\n'
            #     f'Position: x={pos.x:.3f}, y={pos.y:.3f}, z={pos.z:.3f}\n'
            #     f'Orientation: x={ori.x:.3f}, y={ori.y:.3f}, z={ori.z:.3f}, w={ori.w:.3f}'
            # )
    
    def get_marker_pose(self, marker_id):
        """Get the latest pose for a specific marker ID"""
        return self.marker_poses.get(marker_id, None)

    def joint_state_callback(self, joint_states):
        """Store current joint states and update KDL FK."""
        with self.mutex2:
            #input('I amtrtying here')
            for i in range(self.no_of_joints):
                self.joint_states_arr[i] = joint_states.position[i]
            #print('joints are,self.joint_states',self.joint_states_arr)
            #self.mutex2.release()

    def cartesian_to_joint_position(self, target_pose):
        """Convert Cartesian pose to joint positions using KDL."""
        try:
            # Create KDL frame from target pose
            target_frame = PyKDL.Frame(
                PyKDL.Rotation.Quaternion(
                    target_pose.orientation.x,
                    target_pose.orientation.y,
                    target_pose.orientation.z,
                    target_pose.orientation.w
                ),
                PyKDL.Vector(
                    target_pose.position.x,
                    target_pose.position.y,
                    target_pose.position.z
                )
            )
            
            # Create joint array with current positions
            current_joints = PyKDL.JntArray(self.no_of_joints)
            print('current joints first are are',current_joints)
            print('populationed joint_states',self.joint_states_arr)
            for i in range(self.no_of_joints):
                current_joints[i] = self.joint_states_arr[i]
            
            #if len(current_joints) > 0:
            print('Current joints after are',current_joints)
            #input('stop ')
            # Solve IK
            result_joints = PyKDL.JntArray(self.no_of_joints)
            if self.ik_solver.CartToJnt(current_joints, target_frame, result_joints) >= 0:
                return [result_joints[i] for i in range(self.no_of_joints)]
            else:
                self.get_logger().error('IK solution not found')
                return None
                
        except Exception as e:
            self.get_logger().error(f'Failed to convert to joint positions: {str(e)}')
            return None
    def move_only_seventh_axis(self,joint_angle_offset,frame_id="kmriiwa_link_0"):
        #print('Doing thinhgs')
        # Get current joint positions as the other code
        new_joints = PyKDL.JntArray(self.kdl_chain.getNrOfJoints())
        for i in range(self.kdl_chain.getNrOfJoints()):
            new_joints[i] = self.joint_states_arr[i]
        #new_joints = deepcopy(current_joints)
        print('Current joints are',new_joints)
        new_joints[6] += joint_angle_offset
        print('New joints are',new_joints)
        try:
            #input('stop and do this')
            traj_response = self.trajectory_action(new_joints)
            print('Traje_response is',traj_response)
            return self.trajectory_action(new_joints)
            #input('diong this')
            '''
            else:
                self.get_logger().error('IK solution not found')
                return False
            '''
                
        except Exception as e:
            self.get_logger().error(f'Failed to execute joint motion: {str(e)}')
            return False
        
    def move_to_cartesian_pose(self, target_pose, frame_id="kmriiwa_link_0"):
        """Move the arm to a target Cartesian pose using PyKDL."""
        try:
            # Convert target_pose to PyKDL frame
            target_kdl_frame = PyKDL.Frame(
                PyKDL.Rotation.Quaternion(
                    target_pose.orientation.x,
                    target_pose.orientation.y,
                    target_pose.orientation.z,
                    target_pose.orientation.w
                ),
                PyKDL.Vector(
                    target_pose.position.x,
                    target_pose.position.y,
                    target_pose.position.z
                )
            )
            
            # If frame_id is different, transform to base frame
            if frame_id != "kmriiwa_link_0":
                # Create transform from current frame to base frame
                # This is an example - adjust the transform based on your robot's configuration
                transform_to_base = PyKDL.Frame(
                    PyKDL.Rotation.RPY(0, 0, 0),  # Replace with actual rotation
                    PyKDL.Vector(0, 0, 0)         # Replace with actual translation
                )
                
                # Apply transform
                target_kdl_frame = transform_to_base * target_kdl_frame
                
            # Get current joint positions
            current_joints = PyKDL.JntArray(self.kdl_chain.getNrOfJoints())
            for i in range(self.kdl_chain.getNrOfJoints()):
                current_joints[i] = self.joint_states_arr[i]
                
            # Create IK solver if not already created
            if not hasattr(self, 'ik_solver'):
                self.fk_solver = PyKDL.ChainFkSolverPos_recursive(self.kdl_chain)
                self.ik_solver_vel = PyKDL.ChainIkSolverVel_pinv(self.kdl_chain)
                self.ik_solver = PyKDL.ChainIkSolverPos_NR(
                    self.kdl_chain,
                    self.fk_solver,
                    self.ik_solver_vel,
                    100,  # Max iterations
                    1e-6  # Epsilon
                )
                
            # Solve IK
            result_joints = PyKDL.JntArray(self.kdl_chain.getNrOfJoints())
            if self.ik_solver.CartToJnt(current_joints, target_kdl_frame, result_joints) >= 0:
                # Convert to list and execute trajectory
                joint_positions = [result_joints[i] for i in range(self.kdl_chain.getNrOfJoints())]
                print('New joint postions are',joint_positions)
                print('Joint psotions in degrees are',rad2deg(joint_positions).tolist())
                return self.trajectory_action(joint_positions)
            else:
                self.get_logger().error('IK solution not found')
                return False
                
        except Exception as e:
            self.get_logger().error(f'Failed to execute cartesian motion: {str(e)}')
            return False

    def apply_euler_offsets(self,quaternion, roll_offset, pitch_offset, yaw_offset):
        """
        Apply roll, pitch, and yaw offsets to an existing quaternion.
        
        Args:
            quaternion: Existing quaternion as [x, y, z, w]
            roll_offset: Additional rotation around x-axis in radians
            pitch_offset: Additional rotation around y-axis in radians
            yaw_offset: Additional rotation around z-axis in radians
            
        Returns:
            New quaternion with offsets applied, as [x, y, z, w]
        """
        # Create rotation object from quaternion
        r = Rotation.from_quat([quaternion[0], quaternion[1], quaternion[2], quaternion[3]])
        
        # Convert to Euler angles (in the order of 'xyz')
        euler = r.as_euler('xyz')
        
        # Add offsets
        euler[0] += roll_offset
        euler[1] += pitch_offset
        euler[2] += yaw_offset
        
        # Convert back to quaternion
        r_new = Rotation.from_euler('xyz', euler)
        quat_new = r_new.as_quat()
        
        return quat_new
    def get_aruco_orientation(self,vector1,vector2):
       
        """
        Calculate the signed angle between two 2D vectors.
        Positive angle means counterclockwise rotation from vector1 to vector2.
        Crap method for now, because 7-DOF of KUKA IIWA, I am just going to orient the 7th axis,
        since the realsense is mounted above the 7th axis, and can rotate independently
        for other robots, go do some more work do not use this shit primitive methids TODO
        
        Args:
            vector1: First 2D vector [x1, y1]
            vector2: Second 2D vector [x2, y2]
            
        Returns:
            angle in radians
        """
        x1, y1 = vector1
        x2, y2 = vector2
        
        # Using the formula from cross and dot products
        dx = x2 - x1
        dy = y2 - y1
        angle_rad = math.atan2(dy, dx)
        return angle_rad


    
    def get_frame_transform(self, from_frame, to_frame):
        """Get transform between two frames using PyKDL."""
        # This is a simplified example - you would need to implement the actual
        # transform calculation based on your robot's kinematics
        if from_frame == to_frame:
            return PyKDL.Frame()
            
        # Example of creating a transform
        # Replace these values with actual calculations based on your robot
        transform = PyKDL.Frame(
            PyKDL.Rotation.RPY(0, 0, 0),  # Replace with actual rotation
            PyKDL.Vector(0, 0, 0)         # Replace with actual translation
        )
        
        return transform

    # Your existing methods remain unchanged
    def gripper_action(self, action_type):
        msg = String()
        msg.data = ''
        
        if action_type == 'close':
            self.gripper_close_publisher.publish(msg)
        else:
            self.gripper_open_publisher.publish(msg)
            
        self.create_rate(0.5).sleep()
        return True
    '''
    def trajectory_action(self, joint_position_desired):
        with self.mutex1:
            joint_positions = [pos for pos in joint_position_desired]
            traj_msg = JointPosition()
            traj_msg.kmriiwa_joint_1 = joint_positions[0]
            traj_msg.kmriiwa_joint_2 = joint_positions[1]
            traj_msg.kmriiwa_joint_3 = joint_positions[2]
            traj_msg.kmriiwa_joint_4 = joint_positions[3]
            traj_msg.kmriiwa_joint_5 = joint_positions[4]
            traj_msg.kmriiwa_joint_6 = joint_positions[5]
            traj_msg.kmriiwa_joint_7 = joint_positions[6]
            
            self.traj_desired_publisher.publish(traj_msg)
                
            self.create_rate(0.1).sleep()
            
            try:
                return self.traj_response.data == "done"
            except Exception as e:
                self.get_logger().error(f'Failed to send response for trajectory: {str(e)}')
                return False
            pass
    '''

    def trajectory_action(self, joint_position_desired):
        with self.mutex1:
            # Create and send the message
            joint_positions = [pos for pos in joint_position_desired]
            traj_msg = JointPosition()
            traj_msg.kmriiwa_joint_1 = joint_positions[0]
            traj_msg.kmriiwa_joint_2 = joint_positions[1]
            traj_msg.kmriiwa_joint_3 = joint_positions[2]
            traj_msg.kmriiwa_joint_4 = joint_positions[3]
            traj_msg.kmriiwa_joint_5 = joint_positions[4]
            traj_msg.kmriiwa_joint_6 = joint_positions[5]
            traj_msg.kmriiwa_joint_7 = joint_positions[6]
            
            self.traj_desired_publisher.publish(traj_msg)
        
        # Check if joint states reached, so that dont ave to wait forever to finish
        tolerance = 0.001  # radians
        
        # Create a timeout mechanism
        start_time = self.get_clock().now()
        timeout_duration = Duration(seconds=30)  # 10-second timeout
        
        # Loop until joints are at desired position or timeout
        while rclpy.ok():
            # Check if we've reached timeout
            if self.get_clock().now() - start_time > timeout_duration:
                self.get_logger().error('Trajectory action timed out')
                return False
            
            # Get current joint positions
            current_joints = self.joint_states_arr
            
            # Check if all joints are within tolerance of desired position
            all_joints_reached = True
            for i in range(len(joint_positions)):
                if abs(current_joints[i] - joint_positions[i]) > tolerance:
                    all_joints_reached = False
                    break
            
            if all_joints_reached:
                self.get_logger().info('All joints reached desired positions')
                return True
            
            # Spin once to process callbacks and update joint_states_arr
            rclpy.spin_once(self, timeout_sec=0.1)
    def trajectory_action_completed(self, resp_traj):
        self.traj_response = resp_traj
        if resp_traj.data == "done":
            self.get_logger().info('Trajectory completed successfully')

    # Example of how to use Cartesian control with your pick and place
    def cartesian_pick_and_place(self):
        # Example poses - adjust these for your setup
        pick_approach = PoseStamped()
        pick_approach.header.frame_id = "kmriiwa_link_0"
        pick_approach.pose.position.x = 0.5
        pick_approach.pose.position.y = 0.0
        pick_approach.pose.position.z = 0.5
        pick_approach.pose.orientation.w = 1.0

        pick_pose = PoseStamped()
        pick_pose.header.frame_id = "kmriiwa_link_0"
        pick_pose.pose.position.x = 0.5
        pick_pose.pose.position.y = 0.0
        pick_pose.pose.position.z = 0.3
        pick_pose.pose.orientation.w = 1.0

        # Execute pick sequence
        if not self.move_to_cartesian_pose(pick_approach.pose):
            return False
            
        if not self.gripper_action('open'):
            return False
            
        if not self.move_to_cartesian_pose(pick_pose.pose):
            return False
            
        if not self.gripper_action('close'):
            return False
            
        if not self.move_to_cartesian_pose(pick_approach.pose):
            return False

        self.get_logger().info('Cartesian pick successful')
        return True

    def get_current_tcp_pose(self):
        """Get current TCP pose using forward kinematics."""
        #input('I am')

        if not np.any(self.joint_states_arr):
            print("Array contains all zeros")
        else:
            try:
                # Create joint array with current positions
                current_joints = PyKDL.JntArray(self.no_of_joints)
                #self.joint_states_arr = np.array([deg2rad(90),deg2rad(-45),deg2rad(0),deg2rad(90),deg2rad(0),deg2rad(-45),deg2rad(60)]) 
                for i in range(self.no_of_joints):
                    current_joints[i] = self.joint_states_arr[i]

                print('current joitns are',current_joints)
                # Calculate forward kinematics
                current_frame = PyKDL.Frame()
                
                if self.fk_solver.JntToCart(current_joints, current_frame) >= 0:
                    # Convert KDL frame to geometry_msgs.msg.Pose
                    current_pose = PoseStamped()
                    current_pose.header.frame_id = self.base_link
                    current_pose.header.stamp = self.get_clock().now().to_msg()
                    
                    # Position
                    current_pose.pose.position.x = current_frame.p.x()
                    current_pose.pose.position.y = current_frame.p.y()
                    current_pose.pose.position.z = current_frame.p.z()
                    
                    # Convert rotation matrix to quaternion
                    rotation = current_frame.M
                    # PyKDL's rotation matrix to quaternion conversion
                    quat = rotation.GetQuaternion()
                    current_pose.pose.orientation.x = quat[0]
                    current_pose.pose.orientation.y = quat[1]
                    current_pose.pose.orientation.z = quat[2]
                    current_pose.pose.orientation.w = quat[3]

                    print('current_pose is',current_pose)
                    
                    return current_pose
                else:
                    self.get_logger().error('Failed to calculate forward kinematics')
                    return None
                    
            except Exception as e:
                self.get_logger().error(f'Error getting current TCP pose: {str(e)}')
                return None

def main():
    rclpy.init()
    arm_client = ArmManipulationClient()
    #current_pose = arm_client.get_current_tcp_pose()
    
    try:
        # Wait a bit for joint states to be received
        # Create and run a spin_once loop to receive messages
        start_time = time.time()
        while time.time() - start_time < 3.0:  # Wait up to 3 seconds
            rclpy.spin_once(arm_client, timeout_sec=0.1)
            if not np.all(arm_client.joint_states_arr == 0):
                break  # Exit once we get joint states
                
        if np.all(arm_client.joint_states_arr == 0):
            arm_client.get_logger().error("No joint states received!")
            return
        # Get current TCP pose
        current_pose = arm_client.get_current_tcp_pose()
        if current_pose is None:
            arm_client.get_logger().error("Could not get current TCP pose")
            return
            
        arm_client.get_logger().info(f"Current TCP position: x={current_pose.pose.position.x:.3f}, "
                                    f"y={current_pose.pose.position.y:.3f}, "
                                    f"z={current_pose.pose.position.z:.3f}")
        
        
        # Create offset target (move 10cm up in Z)
        target_pose = deepcopy(current_pose)
        


        ## First get the marker pose - example of id1 onw
        #pose_aruco = Pose()
        pose_aruco_position_1 = arm_client.get_marker_pose(1)['position']
        x_pos_aruco_1 = pose_aruco_position_1['x']
        y_pos_aruco_1 = pose_aruco_position_1['y']
        z_pos_aruco_1 = pose_aruco_position_1['z']

        pose_aruco_ori_1 = arm_client.get_marker_pose(1)['orientation']
        x_aruco_ori_1 = pose_aruco_ori_1['x']
        y_aruco_ori_1 = pose_aruco_ori_1['y']
        z_aruco_ori_1 = pose_aruco_ori_1['z']
        w_aruco_ori_1 = pose_aruco_ori_1['w']
        
        aruco_quat_1 = array([x_aruco_ori_1,y_aruco_ori_1,z_aruco_ori_1,w_aruco_ori_1])
        # print('aruco_quaternion_1 is',aruco_quat_1)

        pose_aruco_position_2 = arm_client.get_marker_pose(10)['position']
        x_pos_aruco_2 = pose_aruco_position_2['x']
        y_pos_aruco_2 = pose_aruco_position_2['y']
        z_pos_aruco_2 = pose_aruco_position_2['z']

        pose_aruco_ori_2 = arm_client.get_marker_pose(10)['orientation']
        x_aruco_ori_2 = pose_aruco_ori_2['x']
        y_aruco_ori_2 = pose_aruco_ori_2['y']
        z_aruco_ori_2 = pose_aruco_ori_2['z']
        w_aruco_ori_2 = pose_aruco_ori_2['w']
        aruco_quat_2 = array([x_aruco_ori_2,y_aruco_ori_2,z_aruco_ori_2,w_aruco_ori_2])
        
        # print('aruco_quaternion_2 is',aruco_quat_2)
        
        ## KUTTI VECTOR ALGEBRA

        ## C - camera, A1 - aruco 1, A2- aruco2 
        ## CA2 - CA1 -> CA
        ## CA/2 - > CAvg
        ## Midpoint -> CA_mp = CA1 + CAvg
        CAvg_x = (x_pos_aruco_2 - x_pos_aruco_1)/2.0
        CAvg_y = (y_pos_aruco_2 - y_pos_aruco_1)/2.0
        CAvg_z = (z_pos_aruco_2 - z_pos_aruco_1)/2.0
                
        x_pos_mp_aruco = x_pos_aruco_1 + CAvg_x
        y_pos_mp_aruco = y_pos_aruco_1 + CAvg_y
        z_pos_mp_aruco = z_pos_aruco_1 + CAvg_z




        #x_static_transform_ee_to_cam = 0.0145 # Directly between the center of the optical camera frame to the center
        x_static_transform_ee_to_cam = -0.002 # Directly between the center of the optical camera frame to the center
        #x_static_transform_ee_to_cam = -0.000  ## This is because of the offset from the center of the plate to the holding position
        #y_static_transform_ee_to_cam = -0.080 #-0.107 ## Offset from the center of the wrist to the lens of the camera
        y_static_transform_ee_to_cam = -0.080
        

        aruco_pos1_arr = array([x_pos_aruco_1,y_pos_aruco_1])
        aruco_pos2_arr = array([x_pos_aruco_2,y_pos_aruco_2])


        ## Similarly get the angle of rotaiton with respect to the base


        

        target_pose.pose.position.x = (target_pose.pose.position.x + (x_static_transform_ee_to_cam) + (x_pos_mp_aruco) ) # Testing with ARUCO package
        #target_pose.pose.position.y = (target_pose.pose.position.y - (y_static_transform_ee_to_cam + y_pos_mp_aruco) ) # Testing with ARUCO package
        #target_pose.pose.position.z -= 0.075 # Static offset for testing
        target_pose.pose.position.y = (target_pose.pose.position.y - (y_pos_mp_aruco) - y_static_transform_ee_to_cam   ) # Testing with ARUCO package

        #target_pose.pose.position.z -= 0.175 # Static offset for testing
        #target_pose.pose.position.z -= 0.130 # Static offset for testing
        #target_pose.pose.position.z -= 0.09 # Static offset for testing
        target_pose.pose.position.z -= 0.070 # Static offset for testing - with screw



        arm_client.get_logger().info(f"Moving to: x={target_pose.pose.position.x:.3f}, "
                                        f"y={target_pose.pose.position.y:.3f}, "
                                        f"z={target_pose.pose.position.z:.3f}")
        # qx,qy,qz,qw = arm_client.euler_to_quaternion(0,0,0.750)
        current_quat = [target_pose.pose.orientation.x,target_pose.pose.orientation.y,target_pose.pose.orientation.z,target_pose.pose.orientation.w]
        current_euler = arm_client.quaternion_to_euler(current_quat)
        

        aruco_euler_1 = arm_client.quaternion_to_euler(aruco_quat_1)
        aruco_euler_2 = arm_client.quaternion_to_euler(aruco_quat_2)
        # print('Current euler angles is',current_euler)
        # print('Current euler angles aruco 1 is',aruco_euler_1)
        # print('Current euler angles aruco 2 is',aruco_euler_2)
        result = arm_client.move_to_cartesian_pose(target_pose.pose)
        arm_client.get_logger().info(f"Move completed: {result}")
        #x_static_transform_ee_to_cam = 0.0

        aruco_orientation_angle = arm_client.get_aruco_orientation(aruco_pos1_arr,aruco_pos2_arr)

        #aruco_pose_angle = arm_client.get_seventh_axis_rotation(aruco_orientation)

        # print('Aruco angle now is',aruco_orientation_angle)
        # print('Aruco angle now in degrees is',rad2deg(aruco_orientation_angle))
        #input('stop and check angle')

        
        

        
        sign_of_aruco_orientation = -sign(aruco_orientation_angle)
        const_orientation_offset = 0.785398

        #const_orientation_offset = -sign_of_aruco_orientation*0.15

        #const_orientation_offset = current_euler[1]


        #time.sleep(6.0)
        #input('blocking call now')
        # Execute move
        if result:
            print('Second move now')

            # print('aruco orientation angle is',aruco_orientation_angle)
            result = arm_client.move_only_seventh_axis((aruco_orientation_angle))
            print('result is',result)
            # arm_client.get_logger().info(f"Rotating only the last jiont: {result}")


            # Get current TCP pose agai to check
            current_pose = arm_client.get_current_tcp_pose()
            target_pose = deepcopy(current_pose)
            if current_pose is None:
                arm_client.get_logger().error("Could not get current TCP pose")
                return
                
            arm_client.get_logger().info(f"Current TCP position: x={current_pose.pose.position.x:.3f}, "
                                        f"y={current_pose.pose.position.y:.3f}, "
                                        f"z={current_pose.pose.position.z:.3f}")
            #target_pose.pose.position.z -= 0.022
            target_pose.pose.position.z -= 0.0
            result = arm_client.move_to_cartesian_pose(target_pose.pose)
            arm_client.get_logger().info(f"Move completed: {result}")
        
            '''
            current_pose = arm_client.get_current_tcp_pose()
            target_pose = deepcopy(current_pose)
            if current_pose is None:
                arm_client.get_logger().error("Could not get current TCP pose")
                return
                
            arm_client.get_logger().info(f"Current TCP position: x={current_pose.pose.position.x:.3f}, "
                                        f"y={current_pose.pose.position.y:.3f}, "
                                        f"z={current_pose.pose.position.z:.3f}")
            target_pose.pose.position.z += 0.022
            result = arm_client.move_to_cartesian_pose(target_pose.pose)
            arm_client.get_logger().info(f"Move completed: {result}")
            '''

        
        rclpy.spin(arm_client)
        
    except KeyboardInterrupt:
        arm_client.get_logger().info('Interrupted')
    finally:
        arm_client.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()