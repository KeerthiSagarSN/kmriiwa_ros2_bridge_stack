#!/usr/bin/env python3
import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Pose
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from kmriiwa_msgs.msg import JointPosition
from std_msgs.msg import String
import numpy
from threading import Lock
from std_msgs.msg import Float32, Float32MultiArray
from numpy import deg2rad

class kmr_iiwa_base_move():
    def __init__(self):


        ### Declare all variables here
        ## All joint states are declared here
        self.joint_states_positions = JointState()

        self.a0 = Float32()
        self.a1 = Float32()
        self.a2 = Float32()
        self.a3 = Float32()
        self.a4 = Float32()
        self.a5 = Float32()
        self.a6 = Float32()

        self.mutex1 = Lock()
        self.joint_states_positions.name =  ['kmriiwa_joint_1', 'kmriiwa_joint_2',
                                  'kmriiwa_joint_3', 'kmriiwa_joint_4', 'kmriiwa_joint_5', 'kmriiwa_joint_6','kmriiwa_joint_7']
        self.joint_states_arr = numpy.zeros(7)
        # Get the trajectory response  here
        self.traj_response = String()





        self.client = actionlib.SimpleActionClient('/kmriiwa/move_base',MoveBaseAction)

        # Keep track of the joint states continously
        self.joint_state_subscriber = rospy.Subscriber("kmriiwa/arm/joint_states",JointState,self.joint_state_callback,queue_size=1)

        # Gripper close Publisher
        self.gripper_close_publisher = rospy.Publisher("/kmriiwa/base/command/gripperActionClose",String,queue_size=1)


        # Gripper open Publisher
        self.gripper_open_publisher = rospy.Publisher("/kmriiwa/base/command/gripperActionOpen",String,queue_size=1)


        self.response_trajectory = rospy.Subscriber("/kmriiwa/arm/state/JointPositionReached",String,self.trajectory_action_completed,queue_size=1)

        # Publish the desired trajectory here -> Make this action client later when you have the fucking time!!!
        self.traj_desired_publisher = rospy.Publisher('/kmriiwa/arm/command/JointPosition',JointPosition,queue_size=1)

        self.traj_number = 0

    def joint_state_callback(self,joint_states):
        self.joint_states_arr[0] = joint_states.position[0]
        self.joint_states_arr[1] = joint_states.position[1]
        self.joint_states_arr[2] = joint_states.position[2]
        self.joint_states_arr[3] = joint_states.position[3]
        self.joint_states_arr[4] = joint_states.position[4]
        self.joint_states_arr[5] = joint_states.position[5]
        self.joint_states_arr[6] = joint_states.position[6]


        #print('self.joint_states_positions',joint_states.position)
    def gripper_close_action(self):
        gripper_close_msg = String()
        gripper_close_msg.data=''

        self.gripper_close_publisher.publish(gripper_close_msg)
        # Sleep for few seconds
        rospy.sleep(3.0)
        gripper_close_resp = True
        return gripper_close_resp
    
    def gripper_open_action(self):
        gripper_open_msg = String()
        gripper_open_msg.data=''

        self.gripper_open_publisher.publish(gripper_open_msg)
        # Sleep for few seconds
        rospy.sleep(3.0)
        gripper_open_resp = True
        return gripper_open_resp

    def pick_action(self,start_points,pick_points): ## TODO Worst implementation make gripper open close as service !!!!!

        resp = self.trajectory_action(start_points)

        resp_pick = False
        if resp == True:
            resp1 = self.gripper_open_action()
            if resp1 == True:
                resp2 = self.trajectory_action(pick_points)
                if resp2 == True:
                    resp3 = self.gripper_close_action()
                    if resp3 == True:
                        resp4 = self.trajectory_action(start_points)
                        if resp4 == True:
                            print('Pick action successful')
                            resp_pick = True
                        
        return resp_pick

    def place_action(self,start_points,place_points): ## TODO Worst implementation make gripper open close as service !!!!! and Trajectory as Actionclient

        resp = self.trajectory_action(start_points)
        resp_place = False
        
        if resp == True:
            resp2 = self.trajectory_action(place_points)
            if resp2 == True:
                resp3 = self.gripper_open_action()
                if resp3 == True:
                    resp4 = self.trajectory_action(start_points)

                    if resp4 == True:
                        print('Place action successful')
                        resp_place = True
                    else:
                        resp_places = False
        return resp_place
    
    def inter_points(self,start_points): ## TODO Worst implementation make gripper open close as service !!!!! and Trajectory as Actionclient

        resp = self.trajectory_action(start_points)
        # resp_place = False
        
        if resp == True:
            # self.trajectory_action(start_points)
            print("True")
        #     if resp2 == True:
        #         resp3 = self.gripper_open_action()
        #         if resp3 == True:
        #             resp4 = self.trajectory_action(start_points)

        #             if resp4 == True:
        #                 print('interpoints action successful')
        #                 resp_place = True
        #             else:
        #                 resp_places = False
        return resp



    def trajectory_action_completed(self,resp_traj):
        self.traj_response.data = resp_traj.data
        print('trajectory response',self.traj_response)
        if self.traj_response.data == "done":
            print('i am winning')
        #     self.traj_response.data = ''                
        # if self.traj_response.data == '':
        #     print('i am sleeping')

    def trajectory_action(self,joint_position_desired):
        self.mutex1.acquire()
        
        #joint_position_radians = Float32MultiArray()
        #joint_position_radians.layout = 
        #for i in range(len(joint_position_desired)):
        self.a0.data = deg2rad(joint_position_desired[0])
        self.a1.data = deg2rad(joint_position_desired[1])
        self.a2.data = deg2rad(joint_position_desired[2])
        self.a3.data = deg2rad(joint_position_desired[3])
        self.a4.data = deg2rad(joint_position_desired[4])
        self.a5.data = deg2rad(joint_position_desired[5])
        self.a6.data = deg2rad(joint_position_desired[6])

        
        joint_position_msg = JointPosition()
        joint_position_msg.a1 = self.a0.data
        joint_position_msg.a2 = self.a1.data
        joint_position_msg.a3 = self.a2.data
        joint_position_msg.a4 = self.a3.data
        joint_position_msg.a5 = self.a4.data
        joint_position_msg.a6 = self.a5.data
        joint_position_msg.a7 = self.a6.data
        

        self.traj_desired_publisher.publish(joint_position_msg)
        self.mutex1.release()
        rospy.sleep(10)
        if self.traj_response.data == "done":
            # Do this to get ready for next trajectory point sent
            self.traj_response.data=''
            resp = True
            
        else:
            resp = False
        
        return resp        

            
        
    def start_pick_and_place(self):
        pick_place_conplete = False
        resp_pick_action = False
        resp_approch_action = False
        resp_appoc_place = False
        resp_place_action = False

        start_approch_pick = numpy.array([[90,-31.33,0.0,84.04,0.0,-64.64,60.00]])

        start_robot_points = numpy.array([[90.12,-43.52,0.0,90.43,0.0,-46.07,60.12]])



        robot_points = numpy.array([[90.12,-46.72,0.0,90.55,0.0,-42.74,60.12]])


        start_approch_place = numpy.array([[132.38,-22.53,0.0,95.72,-0.61,-62.21,99.03]])



        start_table_points = numpy.array([[150.77,-63.66,0,24.59,-0.37,-92.35,117.07]])


        table_points = numpy.array([[150.77,-63.69,0,28.48,-0.37,-88.43,117.12]])


        
        for i in range(numpy.shape(start_robot_points)[0]):
            self.traj_number = i

            resp_approch_action = self.inter_points(start_approch_pick[i]) 

            if resp_approch_action == True:
                resp_pick_action = self.pick_action(start_robot_points[i],robot_points[i])
            
                if resp_pick_action == True:
                    resp_approch_action_1 = self.inter_points(start_approch_pick[i])

                    if resp_approch_action_1 == True:
                        resp_appoc_place = self.inter_points(start_approch_place[i])

                        if resp_appoc_place == True:
                            resp_place_action = self.place_action(start_table_points[i], table_points[i])

                            if resp_place_action == True:
                                resp_appoc_place_1 = self.inter_points(start_approch_place[i])

                                if resp_appoc_place_1 == True:
                                    resp_approch_action = self.inter_points(start_approch_pick[i])
                                    print('Pick and place action Successful')
                                    pick_place_conplete = True

                                else:
                                    print('Error somewhere')
                                
                                    pick_place_conplete = False
            return pick_place_conplete
            

        
    def movebase_client(self,x):
        self.client.wait_for_server()
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = float(x[0]) #(0,0 Infront of Keerthi cell -> Map = bay3_v02 )
        goal.target_pose.pose.position.y = float(x[1])
        goal.target_pose.pose.orientation.x = float(x[2])
        goal.target_pose.pose.orientation.y = float(x[3])
        goal.target_pose.pose.orientation.z = float(x[4])
        goal.target_pose.pose.orientation.w = float(x[5])

        self.client.send_goal(goal)
        wait = self.client.wait_for_result()
        if not wait:
            rospy.logerr("Action server not available!")
            rospy.signal_shutdown("Action server not available!")
        else:
            return self.client.get_result() 
          

if __name__ == '__main__':
    
    position_lists_to_station = []
    position_lists_to_base = []
    counter_to_inspection = 0
    flag_pick_place_status = False
    try:
        rospy.init_node('movebase_client_py')

        
        kmr_iiwa_move = kmr_iiwa_base_move()
 
        
        position_lists_to_station.append([6.474208360020238,-0.24058081857955735,0,0,0,1]) # Mid point for 3D Printing cell
        position_lists_to_station.append([8.890382842810087,-1.8686570252329744,0,0,0,1]) #3D Printing Cell

        position_lists_to_station.append([8.890382842810087,0.31547368863465245,0,0,0,1]) #Mid point for Inspection Cell
        position_lists_to_station.append([14.584765190209325,0.8524928794968102,0,0,0,1]) # Mid point for Inspection Cell
        position_lists_to_station.append([14.802961521427815,1.1215917271424243,0,0,1,0]) # Mid point for Inspection Cell
        # position_lists.append([14.702961521427815,1.272299329304691,0,0,1,0]) # Mid point for Inspection Cell
        # position_lists.append([14.702961521427815,1.392299329304691,0,0,1,0]) # Mid point for Inspection Cell
        position_lists_to_station.append([14.702961521427815,1.512299329304691,0,0,1,0]) # Mid point for Inspection Cell
        # Program the cube pick and place
        
        

        #position_lists.append([14.802961521427815,1.392299329304691,0,0,1,0]) # Mid point for Inspection Cell
        position_lists_to_base.append([14.584765190209325,0.8524928794968102,0,0,1,0]) # Mid point for Inspection Cell
        position_lists_to_base.append([11.258698574848232,0.9787469488450701,0,0,1,0]) # Assembly Cell
        ### make all ARM commands above here
        position_lists_to_base.append([6.349122550110111,-0.09777918396093493,0,0,1,0]) # Back to initial point
        position_lists_to_base.append([0.15133437349932619,-0.060470608381884554,0,0,1,0]) # Back to Initial point
        position_lists_to_base.append([0.025138049939463752,-0.0064529444114376205,0,0,0,1]) # Assembly Cell        
        # print(position_lists)
        
        for number_of_positions in range(0,len(position_lists_to_station)):
            positions = (position_lists_to_station[number_of_positions])
            results = kmr_iiwa_move.movebase_client(positions)
            if results:
                rospy.loginfo("Goal execution done!")
                counter_to_inspection += 1
                print(counter_to_inspection)

        if counter_to_inspection == 6:
            flag_pick_place_status = kmr_iiwa_move.start_pick_and_place()
        
        if flag_pick_place_status == True:
            for number_of_positions1 in range(0,len(position_lists_to_base)):
                positions1 = (position_lists_to_base[number_of_positions1])
                results1 = kmr_iiwa_move.movebase_client(positions1)
                if results1:
                    rospy.loginfo("Goal execution base done!")
        else:
            print("pick fail check flag")        

    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")

        # /kmriiwa/base/state/RobotStatus