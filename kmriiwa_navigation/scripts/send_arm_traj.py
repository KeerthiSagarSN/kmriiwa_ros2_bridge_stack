#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped, Pose
from nav2_msgs.action import NavigateToPose
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
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
        
        # # Initialize joint angles
        # self.joint_angles = [Float32() for _ in range(7)]
        
        # # Subscribers
        # self.joint_state_subscriber = self.create_subscription(
        #     JointState,
        #     'kmriiwa/arm/joint_states',
        #     self.joint_state_callback,
        #     1
        # )
        
        # self.response_trajectory = self.create_subscription(
        #     String,
        #     'kmriiwa/arm/state/JointPositionReached',
        #     self.trajectory_action_completed,
        #     1
        # )
        
        # # Publishers
        # self.gripper_close_publisher = self.create_publisher(
        #     String,
        #     'kmriiwa/base/command/gripperActionClose',
        #     1
        # )
        
        # self.gripper_open_publisher = self.create_publisher(
        #     String,
        #     'kmriiwa/base/command/gripperActionOpen',
        #     1
        # )
        
        # self.traj_desired_publisher = self.create_publisher(
        #     JointTrajectory,
        #     'kmriiwa/arm/command/JointPosition',
        #     1
        # )

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

    def joint_state_callback(self, joint_states):
        """Joint states callback"""
        for i in range(7):
            self.joint_states_arr[i] = joint_states.position[i]

    def gripper_action(self, action_type):
        """Handle gripper actions"""
        msg = String()
        msg.data = ''
        
        if action_type == 'close':
            self.gripper_close_publisher.publish(msg)
        else:
            self.gripper_open_publisher.publish(msg)
            
        # Sleep using ROS2 time
        self.create_rate(0.33).sleep()  # 3 second sleep
        return True

    def pick_action(self, start_points, pick_points):
        """Execute pick action sequence"""
        resp = self.trajectory_action(start_points)
        if not resp:
            return False
            
        if not self.gripper_action('open'):
            return False
            
        if not self.trajectory_action(pick_points):
            return False
            
        if not self.gripper_action('close'):
            return False
            
        if not self.trajectory_action(start_points):
            return False
            
        self.get_logger().info('Pick action successful')
        return True

    def place_action(self, start_points, place_points):
        """Execute place action sequence"""
        if not self.trajectory_action(start_points):
            return False
            
        if not self.trajectory_action(place_points):
            return False
            
        if not self.gripper_action('open'):
            return False
            
        if not self.trajectory_action(start_points):
            return False
            
        self.get_logger().info('Place action successful')
        return True

    def trajectory_action(self, joint_position_desired):
        """Execute trajectory action"""
        with self.mutex1:
            # Convert degrees to radians
            joint_positions = [np.deg2rad(pos) for pos in joint_position_desired]
            
            # Create trajectory message
            traj_msg = JointTrajectory()
            traj_msg.joint_names = [f'kmriiwa_joint_{i+1}' for i in range(7)]
            
            point = JointTrajectoryPoint()
            point.positions = joint_positions
            point.time_from_start.sec = 1
            traj_msg.points.append(point)
            
            self.traj_desired_publisher.publish(traj_msg)
            
        # Sleep using ROS2 time
        self.create_rate(0.1).sleep()  # 10 second sleep
        
        return self.traj_response.data == "done"

    def trajectory_action_completed(self, resp_traj):
        """Trajectory completion callback"""
        self.traj_response = resp_traj
        if resp_traj.data == "done":
            self.get_logger().info('Trajectory completed successfully')

def main():
    rclpy.init()
    navigator = NavigationClient()
    
    # IMR ROS1 demo stations
    #[13.792, 2.29008, 0, 0, 1, 0], # Iniital start pose marked near keethi cell
    position_lists_to_station = [        
        [5.9446, 2.29088, 0, 0, 1, 0], # Near corridor in Bay3
        [4.28486, 3.16318, 0, 0, -1, 0.0438], # 3D Printer station 
        [4.3710, 1.15286, 0, 0, 1, 0.0449], # Interp points
        [1.9337, 1.7335, 0, 0, -0.71279, 0.70134] # Assembly station with UR

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