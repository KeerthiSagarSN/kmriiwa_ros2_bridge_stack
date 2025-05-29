#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from nav2_msgs.action import NavigateToPose
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import String, Float32, Float32MultiArray
from geometry_msgs.msg import PoseStamped
import numpy
from threading import Lock
from numpy import deg2rad
from rclpy.duration import Duration

# Assuming kmriiwa_msgs is ported to ROS2
from kmriiwa_msgs.msg import JointPosition

class KmrIiwaBaseMove(Node):
    def __init__(self):
        super().__init__('kmr_iiwa_control_node')
        
        # Create callback group for concurrent callbacks
        self.callback_group = ReentrantCallbackGroup()
        
        # Initialize variables
        self.joint_states_positions = JointState()
        self.a0 = Float32()
        self.a1 = Float32()
        self.a2 = Float32()
        self.a3 = Float32()
        self.a4 = Float32()
        self.a5 = Float32()
        self.a6 = Float32()
        
        self.mutex1 = Lock()
        self.joint_states_positions.name = [
            'kmriiwa_joint_1', 'kmriiwa_joint_2', 'kmriiwa_joint_3',
            'kmriiwa_joint_4', 'kmriiwa_joint_5', 'kmriiwa_joint_6',
            'kmriiwa_joint_7'
        ]
        self.joint_states_arr = numpy.zeros(7)
        self.traj_response = String()
        
        # Initialize action client for navigation
        self.nav_client = ActionClient(
            self,
            NavigateToPose,
            '/kmriiwa/navigate_to_pose',
            callback_group=self.callback_group
        )
        
        # Initialize subscribers
        self.joint_state_subscriber = self.create_subscription(
            JointState,
            'kmriiwa/arm/joint_states',
            self.joint_state_callback,
            1,
            callback_group=self.callback_group
        )
        
        self.response_trajectory = self.create_subscription(
            String,
            '/kmriiwa/arm/state/JointPositionReached',
            self.trajectory_action_completed,
            1,
            callback_group=self.callback_group
        )
        
        # Initialize publishers
        self.gripper_close_publisher = self.create_publisher(
            String,
            '/kmriiwa/base/command/gripperActionClose',
            1
        )
        
        self.gripper_open_publisher = self.create_publisher(
            String,
            '/kmriiwa/base/command/gripperActionOpen',
            1
        )
        
        self.traj_desired_publisher = self.create_publisher(
            JointPosition,
            '/kmriiwa/arm/command/JointPosition',
            1
        )
        
        self.traj_number = 0

    def joint_state_callback(self, joint_states):
        for i in range(7):
            self.joint_states_arr[i] = joint_states.position[i]

    async def gripper_close_action(self):
        gripper_close_msg = String()
        gripper_close_msg.data = ''
        self.gripper_close_publisher.publish(gripper_close_msg)
        await self.sleep(3.0)
        return True

    async def gripper_open_action(self):
        gripper_open_msg = String()
        gripper_open_msg.data = ''
        self.gripper_open_publisher.publish(gripper_open_msg)
        await self.sleep(3.0)
        return True

    async def pick_action(self, start_points, pick_points):
        resp = await self.trajectory_action(start_points)
        if not resp:
            return False

        resp1 = await self.gripper_open_action()
        if not resp1:
            return False

        resp2 = await self.trajectory_action(pick_points)
        if not resp2:
            return False

        resp3 = await self.gripper_close_action()
        if not resp3:
            return False

        resp4 = await self.trajectory_action(start_points)
        if resp4:
            self.get_logger().info('Pick action successful')
            return True
        return False

    async def place_action(self, start_points, place_points):
        resp = await self.trajectory_action(start_points)
        if not resp:
            return False

        resp2 = await self.trajectory_action(place_points)
        if not resp2:
            return False

        resp3 = await self.gripper_open_action()
        if not resp3:
            return False

        resp4 = await self.trajectory_action(start_points)
        if resp4:
            self.get_logger().info('Place action successful')
            return True
        return False

    async def inter_points(self, start_points):
        resp = await self.trajectory_action(start_points)
        if resp:
            self.get_logger().info("Intermediate point reached")
        return resp

    def trajectory_action_completed(self, resp_traj):
        self.traj_response.data = resp_traj.data
        if self.traj_response.data == "done":
            self.get_logger().info('Trajectory completed successfully')

    async def trajectory_action(self, joint_position_desired):
        self.mutex1.acquire()
        try:
            joint_position_msg = JointPosition()
            for i in range(7):
                setattr(joint_position_msg, f'a{i+1}', 
                       deg2rad(joint_position_desired[i]))

            self.traj_desired_publisher.publish(joint_position_msg)
            await self.sleep(10.0)
            
            if self.traj_response.data == "done":
                self.traj_response.data = ''
                return True
            return False
            
        finally:
            self.mutex1.release()

    async def sleep(self, seconds):
        """Helper function for async sleep"""
        await rclpy.time.sleep(seconds)

    async def movebase_client(self, x):
        # Wait for action server
        while not self.nav_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().info('Navigation action server not available, waiting...')

        # Create goal
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = "map"
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        
        # Set position and orientation
        goal.pose.pose.position.x = float(x[0])
        goal.pose.pose.position.y = float(x[1])
        goal.pose.pose.orientation.x = float(x[2])
        goal.pose.pose.orientation.y = float(x[3])
        goal.pose.pose.orientation.z = float(x[4])
        goal.pose.pose.orientation.w = float(x[5])

        # Send goal
        send_goal_future = await self.nav_client.send_goal_async(goal)
        
        if not send_goal_future.accepted:
            self.get_logger().error('Goal rejected')
            return False

        # Wait for result
        result_future = await send_goal_future.get_result_async()
        return result_future.result

    async def start_pick_and_place(self):
        start_approch_pick = numpy.array([[90,-31.33,0.0,84.04,0.0,-64.64,60.00]])
        start_robot_points = numpy.array([[90.12,-43.52,0.0,90.43,0.0,-46.07,60.12]])
        robot_points = numpy.array([[90.12,-46.72,0.0,90.55,0.0,-42.74,60.12]])
        start_approch_place = numpy.array([[132.38,-22.53,0.0,95.72,-0.61,-62.21,99.03]])
        start_table_points = numpy.array([[150.77,-63.66,0,24.59,-0.37,-92.35,117.07]])
        table_points = numpy.array([[150.77,-63.69,0,28.48,-0.37,-88.43,117.12]])

        for i in range(numpy.shape(start_robot_points)[0]):
            self.traj_number = i

            resp_approch_action = await self.inter_points(start_approch_pick[i])
            if not resp_approch_action:
                return False

            resp_pick_action = await self.pick_action(start_robot_points[i], robot_points[i])
            if not resp_pick_action:
                return False

            resp_approch_action_1 = await self.inter_points(start_approch_pick[i])
            if not resp_approch_action_1:
                return False

            resp_appoc_place = await self.inter_points(start_approch_place[i])
            if not resp_appoc_place:
                return False

            resp_place_action = await self.place_action(start_table_points[i], table_points[i])
            if not resp_place_action:
                return False

            resp_appoc_place_1 = await self.inter_points(start_approch_place[i])
            if not resp_appoc_place_1:
                return False

            resp_approch_action = await self.inter_points(start_approch_pick[i])
            if resp_approch_action:
                self.get_logger().info('Pick and place action Successful')
                return True
            
            self.get_logger().error('Error in pick and place sequence')
            return False

async def main():
    rclpy.init()
    node = KmrIiwaBaseMove()
    
    position_lists_to_station = [
        [6.474208360020238, -0.24058081857955735, 0, 0, 0, 1],  # Mid point for 3D Printing cell
        [8.890382842810087, -1.8686570252329744, 0, 0, 0, 1],   # 3D Printing Cell
        [8.890382842810087, 0.31547368863465245, 0, 0, 0, 1],   # Mid point for Inspection Cell
        [14.584765190209325, 0.8524928794968102, 0, 0, 0, 1],   # Mid point for Inspection Cell
        [14.802961521427815, 1.1215917271424243, 0, 0, 1, 0],   # Mid point for Inspection Cell
        [14.702961521427815, 1.512299329304691, 0, 0, 1, 0]     # Mid point for Inspection Cell
    ]

    position_lists_to_base = [
        [14.584765190209325, 0.8524928794968102, 0, 0, 1, 0],   # Mid point for Inspection Cell
        [11.258698574848232, 0.9787469488450701, 0, 0, 1, 0],   # Assembly Cell
        [6.349122550110111, -0.09777918396093493, 0, 0, 1, 0],  # Back to initial point
        [0.15133437349932619, -0.060470608381884554, 0, 0, 1, 0], # Back to Initial point
        [0.025138049939463752, -0.0064529444114376205, 0, 0, 0, 1] # Assembly Cell
    ]

    try:
        # Move to station positions
        counter_to_inspection = 0
        for position in position_lists_to_station:
            result = await node.movebase_client(position)
            if result:
                node.get_logger().info("Goal execution done!")
                counter_to_inspection += 1

        # Execute pick and place if all positions reached
        if counter_to_inspection == 6:
            flag_pick_place_status = await node.start_pick_and_place()
            
            # Return to base if pick and place successful
            if flag_pick_place_status:
                for position in position_lists_to_base:
                    result = await node.movebase_client(position)
                    if result:
                        node.get_logger().info("Goal execution base done!")
            else:
                node.get_logger().error("Pick and place failed")

    except KeyboardInterrupt:
        node.get_logger().info("Navigation test finished.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    import asyncio
    asyncio.run(main())