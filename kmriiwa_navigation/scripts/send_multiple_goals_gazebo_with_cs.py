#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped, Pose
from nav2_msgs.action import NavigateToPose
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import String, Float32, Float32MultiArray, Bool
import numpy as np
from threading import Lock
from rclpy.duration import Duration
from action_msgs.msg import GoalStatus
from rclpy.callback_groups import ReentrantCallbackGroup

class NavigationClient(Node):
    def __init__(self):
        super().__init__('navigation_client')
        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        # Publishers for goal cognition system
        self.goal_pose_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self.critical_goal_pub = self.create_publisher(Bool, '/critical_goal', 10)
        
        # Joint states and variables
        self.joint_states_positions = JointState()
        self.joint_states_arr = np.zeros(7)
        self.mutex1 = Lock()
        self.traj_response = String()
        self.navigation_complete = False
        
    def send_goal_with_criticality(self, x, y, z, is_critical=False, orientation_w=1.0):
        """Send navigation goal with criticality annotation"""
        # First publish the goal pose to trigger goal cognition
        goal_pose_msg = PoseStamped()
        goal_pose_msg.header.frame_id = 'map'
        goal_pose_msg.header.stamp = self.get_clock().now().to_msg()
        goal_pose_msg.pose.position.x = float(x)
        goal_pose_msg.pose.position.y = float(y)
        goal_pose_msg.pose.position.z = 0.0
        goal_pose_msg.pose.orientation.w = orientation_w
        
        # Publish critical flag
        critical_msg = Bool()
        critical_msg.data = is_critical
        
        # Publish both messages
        self.goal_pose_pub.publish(goal_pose_msg)
        self.critical_goal_pub.publish(critical_msg)
        
        self.get_logger().info(f'Published goal ({x}, {y}) with criticality: {is_critical}')
        
        # Small delay to ensure cognition system processes the goal
        rclpy.spin_once(self, timeout_sec=0.1)
        
        # Now send the actual navigation goal
        return self.send_goal(x, y, z)

    def send_waypoint_goal_with_criticality(self, pose_array, is_critical=False):
        """Send navigation goal with full pose array and criticality"""
        # First publish the goal pose to trigger goal cognition
        goal_pose_msg = PoseStamped()
        goal_pose_msg.header.frame_id = 'map'
        goal_pose_msg.header.stamp = self.get_clock().now().to_msg()
        goal_pose_msg.pose.position.x = float(pose_array[0])
        goal_pose_msg.pose.position.y = float(pose_array[1])
        goal_pose_msg.pose.position.z = 0.0
        goal_pose_msg.pose.orientation.x = float(pose_array[2])
        goal_pose_msg.pose.orientation.y = float(pose_array[3])
        goal_pose_msg.pose.orientation.z = float(pose_array[4])
        goal_pose_msg.pose.orientation.w = float(pose_array[5])
        
        # Publish critical flag
        critical_msg = Bool()
        critical_msg.data = is_critical
        
        # Publish both messages
        self.goal_pose_pub.publish(goal_pose_msg)
        self.critical_goal_pub.publish(critical_msg)
        
        self.get_logger().info(f'Published waypoint goal with criticality: {is_critical}')
        
        # Small delay to ensure cognition system processes the goal
        rclpy.spin_once(self, timeout_sec=0.1)
        
        # Now send the actual navigation goal
        return self.send_waypoint_goal(pose_array)

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
    
    # Define goals with criticality annotations
    # Format: [x, y, z, qx, qy, qz, qw, is_critical]
    waypoint_goals = [
        {
            'pose': [26.1305, 16.2752, 0, 0, 0, -0.68892, 0.724838],
            'critical': False,
            'description': 'Near corridor in Bay3'
        },
        {
            'pose': [26.283, 14.6626, 0, 0, 0, -0.0108419, 0.999941],
            'critical': True,  # Mark this as critical
            'description': 'Assembly station with UR - CRITICAL'
        },
        {
            'pose': [25.1867, 24.4881, 0, 0, 0, -0.70744, 1],
            'critical': False,
            'description': 'Final assembly station'
        }
    ]
    
    try:
        # Execute navigation sequence with criticality
        for goal_info in waypoint_goals:
            navigator.get_logger().info(f"Navigating to: {goal_info['description']}")
            
            future = navigator.send_waypoint_goal_with_criticality(
                goal_info['pose'], 
                goal_info['critical']
            )
            rclpy.spin_until_future_complete(navigator, future)
            
            while not navigator.navigation_complete:
                rclpy.spin_once(navigator)
            
            navigator.get_logger().info(f"Completed: {goal_info['description']}")
            
    except KeyboardInterrupt:
        navigator.get_logger().info('Navigation sequence interrupted')
    finally:
        navigator.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()