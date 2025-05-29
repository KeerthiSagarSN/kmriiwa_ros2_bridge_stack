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
from numpy import deg2rad,rad2deg

# from kdl_parser_py import KDL
from kdl_parser_py import urdf
import tf2_ros
from geometry_msgs.msg import PoseStamped, TransformStamped
from tf2_geometry_msgs import do_transform_pose
import time
from copy import deepcopy


############# Only for Testing ######################################################
class ArmManipulationClient(Node):
    def __init__(self):
        super().__init__('arm_manipulation_client')
        
        # Original members
        self.joint_states_positions = JointState()
        self.joint_states_arr = np.zeros(7) ## Numpy array for calaculation throurhgt
        #self.joints_states_arr = np.array([1.50,-0.707,0.0,1.570,0,-0.707,0])
        #self.joint_states_arr = None
        self.mutex1 = Lock()
        self.mutex2 = Lock()
        self.traj_response = String()
        
        # KDL related members
        self.robot_urdf = None
        self.kdl_chain = None
        self.fk_solver = None
        self.ik_solver = None
        self.ik_solver_vel = None

        self.no_of_joints = 0



        
        

        
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
            self.ee_link = "kmriiwa_link_ee"

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
            input('stop ')
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
    def move_to_joint_pose(self,desired_joints):
        
        try:
            joint_positions = [desired_joints[i] for i in range(self.kdl_chain.getNrOfJoints())]
            print('New joint postions are',joint_positions)
            print('Joint psotions in degrees are',rad2deg(joint_positions).tolist())
            return self.trajectory_action(joint_positions)

        except:
            self.get_logger().error('IK solution not found')
            return False


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
            return self.traj_response.data == "done"

    def trajectory_action_completed(self, resp_traj):
        self.traj_response = resp_traj
        if resp_traj.data == "done":
            self.get_logger().info('Trajectory completed successfully')

    # Example of how to use Cartesian control with your pick and place

    def joint_pick_and_place(self):
        # Example poses - adjust these for your setup
        start_approach_pick = deg2rad([[84.42,-41.77,0.0,90.21,0.0,-48.04,54.44]]).tolist()
        start_pick_points_on_robot = deg2rad([[84.42,-48.38,0.0,90.47,0.0,-41.14,54.44]]).tolist()
        robot_clearing_pose_points_1 = deg2rad([[84.42,-30.07,0.0,36.38,0.0,-113.55,54.44]]).tolist()
        robot_clearing_pose_points_2 = deg2rad([[-9.06,-1.10,0.0,73.92,0.0,-104.98,51.55]]).tolist()
        robot_clearing_pose_points_3 = deg2rad([[-90.85,-1.10,0.0,73.92,0.0,-104.98,51.55]]).tolist()
        scan_aruco_pose_points = deg2rad([[-90.12,-43.15,0.0,40.52,0.0,-96.34,52.27]]).tolist()

        # Execute pick sequence
        if not self.move_to_joint_pose(start_approach_pick):
            return False
        if not self.move_to_joint_pose(start_pick_points_on_robot):
            return False
            
        if not self.gripper_action('open'):
            return False
        
        if not self.move_to_joint_pose(robot_clearing_pose_points_1):
            return False
        if not self.move_to_joint_pose(robot_clearing_pose_points_2):
            return False
        if not self.move_to_joint_pose(robot_clearing_pose_points_3):
            return False
        if not self.move_to_joint_pose(scan_aruco_pose_points):
            return False

        self.get_logger().info('Cartesian pick successful')
        return True

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
        input('I am')

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
        target_pose.pose.position.y += 0.03  # Add 10cm in Z
        
        arm_client.get_logger().info(f"Moving to: x={target_pose.pose.position.x:.3f}, "
                                    f"y={target_pose.pose.position.y:.3f}, "
                                    f"z={target_pose.pose.position.z:.3f}")
        
        # Execute move
        result = arm_client.move_to_cartesian_pose(target_pose.pose)
        arm_client.get_logger().info(f"Move completed: {result}")
        
        # Move back to original position
        arm_client.get_logger().info("Moving back to original position...")
        result = arm_client.move_to_cartesian_pose(current_pose.pose)
        arm_client.get_logger().info(f"Return move completed: {result}")
        
        rclpy.spin(arm_client)
        
    except KeyboardInterrupt:
        arm_client.get_logger().info('Interrupted')
    finally:
        arm_client.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()