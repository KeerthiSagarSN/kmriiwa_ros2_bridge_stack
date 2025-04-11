#!/usr/bin/env python3
import sys
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped, Pose
from nav2_msgs.action import NavigateToPose
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
#from kmriiwa_msgs.msg import JointPosition # Similar to the ros1 message as in stoicic;s repo
from std_msgs.msg import String, Float32, Float32MultiArray
import numpy as np
from threading import Lock
from rclpy.duration import Duration
from action_msgs.msg import GoalStatus
from rclpy.callback_groups import ReentrantCallbackGroup



### Get the ArmManipulation Client coder here -> Getting from script, need to make it get from package directly
## SHITTIEST WAY TO DO IMPORT PLEASE FIX THIS IMMEDIATELY- TODO 
# Add the path where the script is installed
sys.path.append('/home/imr/ros2_ws_coresense/src/kmriiwa_ros_stack/kmriiwa_arm_control/scripts/')
from kmriiwa_arm_controller import ArmManipulationClient

class NavigationClient(Node):
    def __init__(self):
        super().__init__('navigation_client')
        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        # Joint states and variables
        self.joint_states_positions = JointState()
        self.joint_states_arr = np.zeros(7)
        self.mutex1 = Lock()
        self.traj_response = String()
        self.navigation_complete = False
        

    def send_goal(self, x, y, z):
        """Send single navigation goal"""
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        
        goal_msg.pose.pose.position.x = float(x)
        goal_msg.pose.pose.position.y = float(y)
        goal_msg.pose.pose.position.z = 0.0
        goal_msg.pose.pose.orientation.w = 1.0

        return self._send_navigation_goal(goal_msg)

    def send_waypoint_goal(self, pose_array):
        """Send navigation goal with full pose array"""
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        
        goal_msg.pose.pose.position.x = float(pose_array[0])
        goal_msg.pose.pose.position.y = float(pose_array[1])
        goal_msg.pose.pose.position.z = 0.0
        goal_msg.pose.pose.orientation.x = float(pose_array[2])
        goal_msg.pose.pose.orientation.y = float(pose_array[3])
        goal_msg.pose.pose.orientation.z = float(pose_array[4])
        goal_msg.pose.pose.orientation.w = float(pose_array[5])

        return self._send_navigation_goal(goal_msg)

    def _send_navigation_goal(self, goal_msg):
        """Common navigation goal sending logic"""
        self._action_client.wait_for_server()
        self.get_logger().info('Sending goal')
        self.navigation_complete = False
        
        send_goal_future = self._action_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.goal_response_callback)
        return send_goal_future

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            return

        self.get_logger().info('Goal accepted')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        status = future.result().status
        result = future.result().result
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('Navigation succeeded')
            self.navigation_complete = True
        else:
            self.get_logger().info(f'Navigation failed with status: {status}')
        return result


def main():
    rclpy.init()
    navigator = NavigationClient()
    arm_client = ArmManipulationClient()
    
    # IMR ROS1 demo stations
    #[13.792, 2.29008, 0, 0, 1, 0], # Iniital start pose marked near keethi cell
    position_lists_to_station = [        
        #[-3.99683, -0.0315, 0, 0, -0.99953, 0.03048694577], # Near corridor in Bay3
        #[4.3, 3.0, 0, 0, -1, 0.0438], # 3D Printer station
        #[-10.9131,-0.9623727,0.0,0.0,0.0,-0.999895,0.0144902],         
        [-7.73722,-0.85485,0.0,0.0,0.0,-0.68938,0.724396], # Assembly station with UR         
        [-7.73722,-0.854851,0.0,0.0,0.0,-0.68938,0.72439], # Move to next station
        [3.54289,0.383681,0.0,0.0,0.0,-0.999889,0.014893], # Exit station                       
        #[1.50, 1.60, 0, 0, 1, 0], # Assembly station with UR               
        #[13.792, 2.75, 0, 0, 1, 0] # teting only pure y-translation motino

    ]
    
    try:
        # Execute navigation sequence
        my_counter = 0
        for position in position_lists_to_station:
            future = navigator.send_waypoint_goal(position)
            rclpy.spin_until_future_complete(navigator, future)
            while not navigator.navigation_complete:
                rclpy.spin_once(navigator)

            # Lets see if we perform arm manipulation hre- is ths blocking call for BT ??? - TODO
            ## Is this shti method>?
            if my_counter == 1:  # After the first waypoint
                print("Navigation to first waypoint complete, starting arm manipulation")
                            # Execute pick and place sequence with the arm
                try:
                    #start_approach_pick = [90,-31.33,0.0,84.04,0.0,-64.64,60.00]
                    #arm_client.inter_points(start_approach_pick)
                    
                    # For pick and place sequence
                    result = arm_client.start_pick_and_place()
                    arm_client.get_logger().info(f"Pick and place completed: {result}")
                    
                    rclpy.spin(arm_client)
                
                except KeyboardInterrupt:
                    arm_client.get_logger().info('Interrupted')
                finally:
                    arm_client.destroy_node()
                    rclpy.shutdown()
            my_counter += 1
            
        # Execute pick and place sequence
        # start_points = [90.12, -43.52, 0.0, 90.43, 0.0, -46.07, 60.12]
        # pick_points = [90.12, -46.72, 0.0, 90.55, 0.0, -42.74, 60.12]
        # navigator.pick_action(start_points, pick_points)
        
    except KeyboardInterrupt:
        navigator.get_logger().info('Navigation sequence interrupted')
    finally:
        navigator.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()