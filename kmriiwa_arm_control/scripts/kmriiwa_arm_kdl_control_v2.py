#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from kmriiwa_msgs.msg import JointPosition
import PyKDL
from urdf_parser_py.urdf import URDF
import os
import subprocess
from ament_index_python.packages import get_package_share_directory
import numpy as np

class ArmManipulationClient(Node):
    def __init__(self):
        super().__init__('arm_manipulation_client')
        
        self.joint_states_arr = np.zeros(7)
        self.kdl_chain = None
        self.fk_solver = None
        self.ik_solver = None
        
        # Setup KDL
        self.setup_kdl()

    def process_xacro_to_urdf(self):
        """Convert xacro to URDF using command line tools."""
        try:
            # Get path to xacro file
            kmriiwa_description_dir = get_package_share_directory('kmriiwa_description')
            xacro_file = os.path.join(kmriiwa_description_dir, 'urdf', 'robot', 'kmriiwa.urdf.xacro')
            #robot_extras = os.path.join(kmriiwa_description_dir, 'urdf', 'robot', 'empty.xacro')
            
            # Create xacro process command
            cmd = ['xacro', xacro_file]
            
            # Run xacro processor
            process = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
            urdf_str, error = process.communicate()
            
            if process.returncode != 0:
                self.get_logger().error(f'Xacro processing failed: {error.decode()}')
                return None
                
            return urdf_str.decode('utf-8')
            
        except Exception as e:
            self.get_logger().error(f'Failed to process xacro: {str(e)}')
            return None

    def setup_kdl(self):
        """Setup KDL chain from processed URDF."""
        try:
            # First get URDF string from xacro
            urdf_str = self.process_xacro_to_urdf()
            if urdf_str is None:
                raise Exception("Failed to get URDF from xacro")
            
            # Parse URDF string
            robot = URDF.from_xml_string(urdf_str)

            print('robot is',robot)
            
            # Get KDL tree
            kdl_tree = robot.get_kdl_tree()
            
            # Extract chain
            base_link = "kmriiwa_arm_link_0"
            end_link = "kmriiwa_arm_link_ee"
            self.kdl_chain = kdl_tree.getChain(base_link, end_link)
            
            # Create solvers
            self.fk_solver = PyKDL.ChainFkSolverPos_recursive(self.kdl_chain)
            self.ik_solver_vel = PyKDL.ChainIkSolverVel_pinv(self.kdl_chain)
            self.ik_solver = PyKDL.ChainIkSolverPos_NR(
                self.kdl_chain,
                self.fk_solver,
                self.ik_solver_vel,
                100,  # Max iterations
                1e-6  # Epsilon
            )
            
            # Print chain information
            self.get_logger().info(f'Successfully created KDL chain')
            self.get_logger().info(f'Number of joints: {self.kdl_chain.getNrOfJoints()}')
            #self.get_logger().info(f'Number of segments: {self.kdl_chain.getNrOfSegments()}')
            
            # Print joint names from URDF
            joint_names = [joint.name for joint in robot.joints if joint.type != 'fixed']
            self.get_logger().info(f'Active joints: {joint_names}')
            
        except Exception as e:
            self.get_logger().error(f'Failed to setup KDL: {str(e)}')
            raise

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
            current_joints = PyKDL.JntArray(self.kdl_chain.getNrOfJoints())
            for i in range(self.kdl_chain.getNrOfJoints()):
                current_joints[i] = self.joint_states_arr[i]
            
            # Solve IK
            result_joints = PyKDL.JntArray(self.kdl_chain.getNrOfJoints())
            if self.ik_solver.CartToJnt(current_joints, target_frame, result_joints) >= 0:
                return [result_joints[i] for i in range(self.kdl_chain.getNrOfJoints())]
            else:
                self.get_logger().error('IK solution not found')
                return None
                
        except Exception as e:
            self.get_logger().error(f'Failed to convert to joint positions: {str(e)}')
            return None

def main():
    rclpy.init()
    
    try:
        arm_client = ArmManipulationClient()
        rclpy.spin(arm_client)
    except Exception as e:
        print(f'Error: {str(e)}')
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()