#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.duration import Duration


## Template file to try 
class NavigationClient(Node):
    def __init__(self):
        super().__init__('navigation_client')
        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

    def send_goal(self, x, y, z):
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        
        # Goal pose, getting using Estimate pose
        goal_msg.pose.pose.position.x = float(x)
        goal_msg.pose.pose.position.y = float(y)
        goal_msg.pose.pose.position.z = 0.0
        goal_msg.pose.pose.orientation.w = 1.0

        # Wait for action server
        self._action_client.wait_for_server()
        
        # Send goal to nav2 
        self.get_logger().info('Sending goal')
        self._send_goal_future = self._action_client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            return

        self.get_logger().info('Goal accepted')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info('Goal completed')

def main():
    rclpy.init()
    navigator = NavigationClient()
    
    # Send goal - Testing thisfrom random place, seems to be ok
    navigator.send_goal(5.63354778289795, 2.2772679328918457, 0.0)  # Move forward # x -   10.988336563110352 , y - 2.445468, z - 0
    
    rclpy.spin(navigator)
    
    navigator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()