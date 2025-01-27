#!/usr/bin/env python3
import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from rclpy.duration import Duration

def main():
    rclpy.init()
    navigator = BasicNavigator(
        namespace='kmriiwa',
        remappings=[
            ('cmd_vel', 'base/command/cmd_vel'),
        ]
    )
    
    # Set waypoints
    waypoints = [
        [0.60, 1.0, 0.0],  # [x, y, yaw]
        #[2.0, 0.0, 1.57],
        #[0.0, 2.0, -1.57],
        #[0.0, 0.0, 0.0]
    ]
    
    # Convert waypoints to PoseStamped messages
    pose_goals = []
    for wp in waypoints:
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = navigator.get_clock().now().to_msg()
        pose.pose.position.x = wp[0]
        pose.pose.position.y = wp[1]
        pose.pose.position.z = 0.0
        # You'll need to convert yaw to quaternion here
        pose.pose.orientation.w = 1.0
        pose_goals.append(pose)

    # Navigate through poses
    navigator.goThroughPoses(pose_goals)

    while not navigator.isTaskComplete():
        feedback = navigator.getFeedback()
        if feedback:
            print(f'Distance remaining: {feedback.distance_remaining:.2f} meters')

    result = navigator.getResult()
    if result == TaskResult.SUCCEEDED:
        print('Navigation succeeded!')
    elif result == TaskResult.CANCELED:
        print('Navigation was canceled!')
    else:
        print('Navigation failed!')

    navigator.lifecycleShutdown()
    rclpy.shutdown()

if __name__ == '__main__':
    main()