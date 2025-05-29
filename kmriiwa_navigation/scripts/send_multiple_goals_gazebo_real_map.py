#!/usr/bin/env python3
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
    
    # IMR ROS1 demo stations
    #[13.792, 2.29008, 0, 0, 1, 0], # Iniital start pose marked near keethi cell
    position_lists_to_station = [        
        [26.1305, 16.2752, 0, 0, 0, -0.68892, 0.724838], # Near corridor in Bay3
        #[-10.96604, -0.5754522, 0, 0, -0.7000717, 0.71407], # 3D Printer station         
        #[-8.11755, -0.7410191, 0.0, 0, -0.681899, 0.731445], # Assembly station with UR         
        [26.283, 14.6626, 0, 0, 0, -0.0108419, 0.999941], # Assembly station with UR                       
        [25.1867, 24.4881, 0,0, 0, -0.70744, 1], # Assembly station with UR               
        

    ]
    
    try:
        # Execute navigation sequence
        for position in position_lists_to_station:
            future = navigator.send_waypoint_goal(position)
            rclpy.spin_until_future_complete(navigator, future)
            while not navigator.navigation_complete:
                rclpy.spin_once(navigator)
            
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