#!/usr/bin/env python

import rospy

from sensor_msgs.msg import JointState
from std_msgs.msg import Header
import math

# #Dimensiones de la herramienta y el brazo
# longTool= 0.0255
# armLenghts[5] = {0.047, 0.0145, 0.025, 0.0675, (0.0745+longTool)}
# #Dimensiones de la pierna
# LegLenghts[5] = {0.0385, 0.031, 0.0145, 0.075, 0.0295}

class movement:
    def __init__(self):
        rospy.init_node("motion_control")
        self.r = rospy.Rate(10)
        self.pub = rospy.Publisher('/joint_states', JointState, queue_size=1)

    def loop(self):
        self.main()
        self.r.sleep()


    def main(self):

        g30=0.5235987756
        g45=0.7853981634
        dtr=math.pi/180


        time=0.8
        walkingtime=0.5

        self.joints_states = JointState()
        self.joints_states.header = Header()
        self.joints_states.header.stamp = rospy.Time.now()
        self.joints_states.name = ["joint_0_1","joint_0_2","joint_0_3","joint_0_4","joint_0_5","joint_0_6","joint_0_7","joint_0_8","joint_0_9","joint_0_10","joint_0_11","joint_0_12","joint_0_13","joint_0_14","joint_0_15","joint_0_16","joint_0_17","joint_0_18","joint_0_19","joint_0_20","joint_0_21"]
        # right 1:shoulder, 3:upper, 5:below (positive)
        # left 2:shoulder, 4:upper, 6:below (negative)
        # right 9, 11, 13, 15, 17
        # left 10, 12, 14, 16, 18
        number = 0
        while not rospy.is_shutdown():
            
            # number = input ("Enter number: ")
            # if (number == 0): #walkready
                self.joint_position_state=[
                    0*dtr, 0*dtr, #1 2 shoulder
                    0*dtr, 0*dtr, #3 4 upper
                    -90*dtr, -90*dtr, #5 6 below
                    0*dtr, 0*dtr, #7 8 not yet
                    0*dtr, 0*dtr, #9 10 hip yaw (in positive)
                    -35*dtr, -35*dtr, #11 12 hip front back
                    -40*dtr, -40*dtr, #13 14
                    0*dtr, 0*dtr, #15 16 pitch (front negative)
                    0*dtr, 0*dtr, #17 18 yaw (out negative)
                    0*dtr, 0*dtr, 0*dtr] #19 20 21
                self.joints_states.position = self.joint_position_state
                self.pub.publish(self.joints_states)
                rospy.sleep(time)
                print("walkready")
            #     number +=1
            # else
                self.joint_position_state=[
                    0*dtr, 0*dtr, #1 2 shoulder pitch
                    0*dtr, 0*dtr, #3 4 shoulder roll
                    -90*dtr, 0*dtr, #5 6 below
                    0*dtr, 0*dtr, #7 8 not yet
                    0*dtr, 0*dtr, #9 10 hip roll (in positive)
                    -80*dtr, -58*dtr, #11 12 hip pitch
                    -50*dtr, -25*dtr, #13 14 knee
                    -30*dtr, 0*dtr, #15 16 ankle pitch (front negative)
                    0*dtr, 0*dtr, #17 18  ankle roll (out negative)
                    0*dtr, 0*dtr, 0*dtr] #19 20 21 ankle roll
                self.joints_states.position = self.joint_position_state
                self.pub.publish(self.joints_states)
                rospy.sleep(time)
                print("walk")
                self.joint_position_state=[
                    0*dtr, 0*dtr, #1 2 shoulder
                    0*dtr, 0*dtr, #3 4 upper
                    -90*dtr, -90*dtr, #5 6 below
                    0*dtr, 0*dtr, #7 8 not yet
                    0*dtr, 0*dtr, #9 10 hip yaw (in positive)
                    -20*dtr, 0*dtr, #11 12 hip front back
                    -8*dtr, -0*dtr, #13 14
                    0*dtr, 0*dtr, #15 16 pitch (front negative)
                    0*dtr, 0*dtr, #17 18 yaw (out negative)
                    0*dtr, 0*dtr, 0*dtr] #19 20 21
                self.joints_states.position = self.joint_position_state
                self.pub.publish(self.joints_states)
                rospy.sleep(time)
                self.joint_position_state=[
                    0*dtr, 0*dtr, #1 2 shoulder pitch
                    0*dtr, 0*dtr, #3 4 shoulder roll
                    0*dtr, -90*dtr, #5 6 below
                    0*dtr, 0*dtr, #7 8 not yet
                    -0*dtr, 0*dtr, #9 10 hip roll (in positive)
                    -58*dtr, -80*dtr, #11 12 hip pitch
                    -25*dtr, -50*dtr, #13 14 knee
                    0*dtr, -30*dtr, #15 16 ankle pitch (front negative)
                    0*dtr, 0*dtr, #17 18  ankle roll (out negative)
                    0*dtr, 0*dtr, 0*dtr] #19 20 21 
                self.joints_states.position = self.joint_position_state
                self.pub.publish(self.joints_states)
                rospy.sleep(time)
                print("walk 2")
                self.joint_position_state=[
                    0*dtr, 0*dtr, #1 2 shoulder
                    0*dtr, 0*dtr, #3 4 upper
                    -90*dtr, -90*dtr, #5 6 below
                    0*dtr, 0*dtr, #7 8 not yet
                    0*dtr, 0*dtr, #9 10 hip yaw (in positive)
                    -20*dtr, 0*dtr, #11 12 hip front back
                    -0*dtr, -8*dtr, #13 14
                    0*dtr, 0*dtr, #15 16 pitch (front negative)
                    0*dtr, 0*dtr, #17 18 yaw (out negative)
                    0*dtr, 0*dtr, 0*dtr] #19 20 21
                self.joints_states.position = self.joint_position_state
                self.pub.publish(self.joints_states)
                rospy.sleep(time)
                number += 1
            
            # if (number==3): 
            #     self.joint_position_state=[
            #         30*dtr, -30*dtr, #1 2 shoulder pitch
            #         0*dtr, 0*dtr, #3 4 shoulder roll
            #         0*dtr, 0*dtr, #5 6 below
            #         0*dtr, 0*dtr, #7 8 not yet
            #         0*dtr, 0*dtr, #9 10 hip roll (in positive)
            #         -68*dtr, -68*dtr, #11 12 hip pitch
            #         -30*dtr, -30*dtr, #13 14 knee
            #         -5*dtr, -5*dtr, #15 16 ankle pitch (front negative)
            #         0*dtr, 0*dtr, #17 18  ankle roll (out negative)
            #         0*dtr, 0*dtr, 0*dtr] #19 20 21 ankle roll
            #     self.joints_states.position = self.joint_position_state
            #     self.pub.publish(self.joints_states)
                # rospy.sleep(time)
            # if (number==3):
            #     self.joint_position_state=[-0.1,0.1,-0.2,0.2,0.6,-0.6,0,0,     -g30/3,-g30/3,0,0,0,0,0,0,-g30/3,-g30/3,    0,0,0]
            #     self.joints_states.position = self.joint_position_state
            #     self.pub.publish(self.joints_states)
            #     rospy.sleep(time)
            #     self.joint_position_state=[-0.1,0.3,-0.2,0.5,0.6,-0.5,0,0,     -g30/3,-g30/3,-g30,0,-g30,0,         0,0,-g30/3,-g30/3,    0,0,0]
            #     self.joints_states.position = self.joint_position_state
            #     self.pub.publish(self.joints_states)
            #     rospy.sleep(time)
            #     self.joint_position_state=[-0.1,0.3,-0.2,0.5,0.6,-0.5,0,0,     -g30/3,-g30/3,-g30,-g30/3,-g30,0,0,g30/3,-g30/3,-g30/3,    0,0,0]
            #     self.joints_states.position = self.joint_position_state
            #     self.pub.publish(self.joints_states)
            #     rospy.sleep(time)
            #     self.joint_position_state=[-0.1,0.1,-0.2,0.2,0.6,-0.6,0,0,     0,0,-g30,-g30/3,-g30,0,     0,g30/3,0,0,    0,0,0]
            #     self.joints_states.position = self.joint_position_state
            #     self.pub.publish(self.joints_states)
            #     rospy.sleep(time)

                
            # if (number==4):
            #     self.joint_position_state=[-0.1,0.3,-0.2,0.5,0.6,-0.5,0,0,     g30/3,g30/3,-g30*1/3,g30/3,-g30*3/3,g30*3/3,     -g30*2/3,g30*2/3,g30/3,g30/3,    0,0,0]
            #     self.joints_states.position = self.joint_position_state
            #     self.pub.publish(self.joints_states)
            #     rospy.sleep(time)
            #     self.joint_position_state=[-0.1,0.3,-0.2,0.5,0.6,-0.5,0,0,     g30/3,g30/3,0,g30,0,g30,     0,0,g30/3,g30/3,    0,0,0]
            #     self.joints_states.position = self.joint_position_state
            #     self.pub.publish(self.joints_states)
            #     rospy.sleep(time)
            #     self.joint_position_state=[-0.1,0.3,-0.2,0.5,0.6,-0.5,0,0,     g30/3,g30/3,g30*1/3,g30,0,g30,     -g30*1/3,0,g30/3,g30/3,    0,0,0]
            #     self.joints_states.position = self.joint_position_state
            #     self.pub.publish(self.joints_states)
            #     rospy.sleep(time)
            #     self.joint_position_state=[-0.1,0.3,-0.2,0.5,0.6,-0.5,0,0,     -g30/3,-g30/3,0,g30,0,g30,     0,g30*1/3,-g30/3,-g30/3,    0,0,0]
            #     self.joints_states.position = self.joint_position_state
            #     self.pub.publish(self.joints_states)
            #     rospy.sleep(time)
            #     self.joint_position_state=[-0.1,0.1,-0.2,0.2,0.6,-0.6,0,0,     -g30/3,-g30/3,0,0,0,0,           0,0,-g30/3,-g30/3,    0,0,0]
            #     self.joints_states.position = self.joint_position_state
            #     self.pub.publish(self.joints_states)
            #     rospy.sleep(time)
            # if (number==4):
            #     self.joint_position_state=[0.1,-0.3,0.2,-0.5,-0.6,0.5,0,0,     g30/3,g30/3,g30,g30/3,g30,0,0,-g30/3,g30/3,g30/3,    0,0,0]
            #     self.joints_states.position = self.joint_position_state
            #     self.pub.publish(self.joints_states)
            #     rospy.sleep(time)		
            # if (number==5):
            #     self.joint_position_state=[0.1,-0.3,0.2,-0.5,-0.6,0.5,0,0,     0,0,g30,g30/3,g30,0,     0,-g30/3,0,0,    0,0,0]
            #     self.joints_states.position = self.joint_position_state
            #     self.pub.publish(self.joints_states)
            #     rospy.sleep(time)	
            # if (number==6):
            #     self.joint_position_state=[0.1,-0.3,0.2,-0.5,-0.6,0.5,0,0,     -g30/3,-g30/3,g30,g30/3,g30,0,     g30/3,-g30/3,-g30/3,-g30/3,    0,0,0]
            #     self.joints_states.position = self.joint_position_state
            #     self.pub.publish(self.joints_states)
            #     rospy.sleep(time)	
            #     self.joint_position_state=[0.1,-0.3,0.2,-0.5,-0.6,0.5,0,0,     -g30/3,-g30/3,0,g30/2,0,0,0,0,-g30/3,-g30/3,    0,0,0]
            #     self.joints_states.position = self.joint_position_state
            #     self.pub.publish(self.joints_states)
            #     rospy.sleep(time)
            # if (number==7):
            #     # segundo paso
            #     joint_position_state=[0.5,-0.5,0.5,-0.5,0.5,-0.5,-0.5,0.5,0,0,0,0,0,0,0,0,0,0,0,0,0]
            # if (number==8):
            #     # segundo paso
            #     joint_position_state=[0.5,-0.5,0.5,-0.5,0.5,-0.5,-0.5,0.5,0,-0.3,0,0,0,0,0,0,0,0,0,0,0]
            # if (number==9):
            #     # segundo paso
            #     joint_position_state=[0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
            # BACKWARD 2 1
                self.joint_position_state=[
                    0*dtr, 0*dtr, #1 2 shoulder pitch
                    0*dtr, 0*dtr, #3 4 shoulder roll
                    0*dtr, 0*dtr, #5 6 below
                    0*dtr, 0*dtr, #7 8 not yet
                    0*dtr, 0*dtr, #9 10 hip roll (in positive)
                    -80*dtr, -80*dtr, #11 12 hip pitch
                    -50*dtr, -50*dtr, #13 14 knee
                    -30*dtr, -30*dtr, #15 16 ankle pitch (front negative)
                    0*dtr, 0*dtr, #17 18  ankle roll (out negative)
                    0*dtr, 0*dtr, 0*dtr] #19 20 21 ankle roll
                self.joints_states.position = self.joint_position_state
                self.pub.publish(self.joints_states)
                rospy.sleep(time)
                print("1")
                self.joint_position_state=[
                    0*dtr, 0*dtr, #1 2 shoulder
                    0*dtr, 0*dtr, #3 4 upper
                    -90*dtr, -90*dtr, #5 6 below
                    0*dtr, 0*dtr, #7 8 not yet
                    0*dtr, 0*dtr, #9 10 hip yaw (in positive)
                    -35*dtr, -35*dtr, #11 12 hip front back
                    -40*dtr, -40*dtr, #13 14
                    0*dtr, 0*dtr, #15 16 pitch (front negative)
                    0*dtr, 0*dtr, #17 18 yaw (out negative)
                    0*dtr, 0*dtr, 0*dtr] #19 20 21
                self.joints_states.position = self.joint_position_state
                self.pub.publish(self.joints_states)
                rospy.sleep(time)
                print("2")
if __name__ == '__main__':
    movement= movement()
    while not rospy.is_shutdown():
        movement.main() 