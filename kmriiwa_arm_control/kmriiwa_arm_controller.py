#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from kmriiwa_msgs.msg import JointPosition
import numpy as np
from threading import Lock

class ArmManipulationClient(Node):
   def __init__(self):
       super().__init__('arm_manipulation_client')
       
       self.joint_states_positions = JointState()
       self.joint_states_arr = np.zeros(7)
       self.mutex1 = Lock()
       self.traj_response = String()
       
       self.joint_state_subscriber = self.create_subscription(
           JointState,
           'kmriiwa/arm/joint_states',
           self.joint_state_callback,
           10
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

   def joint_state_callback(self, joint_states):
       for i in range(7):
           self.joint_states_arr[i] = joint_states.position[i]

   def gripper_action(self, action_type):
       msg = String()
       msg.data = ''
       
       if action_type == 'close':
           self.gripper_close_publisher.publish(msg)
       else:
           self.gripper_open_publisher.publish(msg)
           
       self.create_rate(0.5).sleep()
       return True

   def pick_action(self, start_points, pick_points):
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

   def inter_points(self, start_points):
       resp = self.trajectory_action(start_points)
       if resp:
           self.get_logger().info("Intermediate point reached")
       return resp

   def trajectory_action(self, joint_position_desired):
       with self.mutex1:
           joint_positions = [np.deg2rad(pos) for pos in joint_position_desired]
           traj_msg = JointPosition()
           traj_msg.kmriiwa_joint_1 = joint_positions[0]
           traj_msg.kmriiwa_joint_2 = joint_positions[1]
           traj_msg.kmriiwa_joint_3 = joint_positions[2]
           traj_msg.kmriiwa_joint_4 = joint_positions[3]
           traj_msg.kmriiwa_joint_5 = joint_positions[4]
           traj_msg.kmriiwa_joint_6 = joint_positions[5]
           traj_msg.kmriiwa_joint_7 = joint_positions[6]
           
           self.traj_desired_publisher.publish(traj_msg)
           
       #self.create_rate(0.5).sleep()
       return self.traj_response.data == "done"

   def trajectory_action_completed(self, resp_traj):
       self.traj_response = resp_traj
       if resp_traj.data == "done":
           self.get_logger().info('Trajectory completed successfully')
           
   def start_pick_and_place(self):
       pick_place_complete = False
       
       # Define waypoints
       #home_position_joints = [[90.0,-45.0,0.0,90.0,0.0,-45.0,60.0]]
       start_approach_pick = [[84.42,-41.77,0.0,90.21,0.0,-48.04,54.44]]
       start_pick_points_on_robot = [[84.42,-48.38,0.0,90.47,0.0,-41.14,54.44]]
       robot_clearing_pose_points_1 = [[84.42,-30.07,0.0,36.38,0.0,-113.55,54.44]]
       robot_clearing_pose_points_2 = [[-9.06,-1.10,0.0,73.92,0.0,-104.98,51.55]]
       robot_clearing_pose_points_3 = [[-90.85,-1.10,0.0,73.92,0.0,-104.98,51.55]]
       scan_aruco_pose_points = [[-90.12,-43.15,0.0,40.52,0.0,-96.34,52.27]]

       # Execute sequence
       for i in range(1):                    
           
           if not self.inter_points(start_approach_pick[i]):
               return False
               
           if not self.pick_action(start_approach_pick[i], start_pick_points_on_robot):
               return False
               
           if not self.inter_points(start_approach_pick[i]):
               return False
               
           if not self.inter_points(robot_clearing_pose_points_1[i]):
               return False
        
           if not self.inter_points(robot_clearing_pose_points_2[i]):
               return False

           if not self.inter_points(robot_clearing_pose_points_3[i]):
               return False

           if not self.inter_points(scan_aruco_pose_points[i]):
               return False

        #        if not self.inter_points(robot_clearing_pose_points_1[i]):
        #        return False
        #    if not self.place_action(start_table_points[i], table_points[i]):
        #        return False
               
        #    if not self.inter_points(start_approach_place[i]):
        #        return False
               
        #    if not self.inter_points(start_approach_pick[i]):
        #        return False
               
       self.get_logger().info('Pick and place action Successful')
       pick_place_complete = True

       return pick_place_complete



def main():
   rclpy.init()
   arm_client = ArmManipulationClient()
   
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

if __name__ == '__main__':
   main()