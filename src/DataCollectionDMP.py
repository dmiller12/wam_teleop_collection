#!/usr/bin/python3


from operator import truediv
import numpy as np
import rospy 

from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from wam_msgs.msg import RTJointPos, RTJointVel
from wam_srvs.srv import JointMove
from wam_srvs.srv import Hold
from std_srvs.srv import Empty

import json
#import pickle
import os
import rosservice
#import pygame
#import keyboard
import glob

joint_state_data = []

EE_pose_data = []

p = 0

key_pressed = []

POS_READY = [
    0.002227924477643431, 
    -0.1490540623980915, 
    -0.04214558734519736, 
    1.6803055108189549, 
    0.06452207850075688, 
    -0.06341508205589094, 
    0.01366506663019359,
]

#pygame.init()
#screen = pygame.display.set_mode((640, 480))

def go_ready_pos():
        """Move WAM to a desired ready position.
        """
        joint_move(POS_READY)

def joint_move(pos_goal: np.ndarray):
        """Move WAM to a desired position.
        q is a numpy array of length 7 that specifies the joint angles
        """
  # Communicate with /wam/joint_move service on control computer
        rospy.wait_for_service('/wam/joint_move')
        try:
            #print('found service')
            joint_move_service = rospy.ServiceProxy('/wam/joint_move', JointMove)
            joint_move_service(pos_goal)
            #print('called move_q')
        except rospy.ServiceException as e:
            print("Service call failed: %s" % e)


def joint_vel_cmd(vel_goal: np.ndarray,
                  jnt_vel_pub):
        msg = RTJointVel()
        # Publish to ROS
        msg.velocities = vel_goal
        #print(rospy.get_time())
        jnt_vel_pub.publish(msg)

def clip_velocity(vel, max_norm):
        vel_norm = np.linalg.norm(vel)
        if vel_norm > max_norm:
            print("clipped vel")
            return vel/vel_norm*max_norm # velocity rescaled to have max norm
        else:
            return vel

def joint_pos_cmd(pos_goal: np.ndarray, jnt_pos_pub):
                      
        msg = RTJointPos()
        # Publish to ROSfrom sensor_msgs.msg import JointState
        msg.joints = pos_goal
        msg.rate_limits = np.array([500.0]*7)
        jnt_pos_pub.publish(msg)

'''
def init_joint_states_listener():
        """Set up joint_states listener from WAM control computer.
        """
        rospy.Subscriber('/wam/joint_states', JointState, callback_joint_state)
        
def init_EE_pose_listener():
        """Set up EE_pose listener from WAM control computer.
        """
        rospy.Subscriber('/wam/pose', PoseStamped, callback_EE_pose)

def init_key_pose_listener():
        """Set up keyobard listener from WAM control computer.
        """ 
        rospy.Subscriber('keyboard_command', String, callback_keyboard)
        
def callback_joint_state(data):
    #global joint_state_final
    #rospy.loginfo(rospy.get_caller_id() + ' I heard %s', data.position)
    #if self.pos == []: # first time we hear the joint states
    #    rospy.loginfo(rospy.get_caller_id() + ' VS system is reading joint states.')
    # print(data.header.stamp.secs)
    # print(list(data.position))
    # print(list(data.velocity))
    joint_state = {'time': data.header.stamp.secs+data.header.stamp.nsecs*10^(-9),
			   'position' : data.position,
			   'velocity' : data.velocity,
			   'effort' : data.effort}
    joint_state_data.append(joint_state)
    
def callback_EE_pose(data):
    #global EE_pose_final
    EE_pose = {'position' : (data.pose.position.x, data.pose.position.y, data.pose.position.z),
			   #'orientation' : data.pose.orientation}
			   'orientation': (data.pose.orientation.x, data.pose.orientation.y, data.pose.orientation.z,  data.pose.orientation.w)}
    #EE_pose_final = EE_pose
    EE_pose_data.append(EE_pose)

def callback_keyboard(data):
    key_pressed = data
    trajs = len(glob.glob('/home/robot/DMP/DMP_data_joint*.json'))
    print("{} trajectories exist".format(trajs))

    if key_pressed == 'p':  # if key 'q' is pressed 
            print('Pick Trj Collected')
            out_file1 = open("/home/robot/DMP_data_joint_pick_{}.json".format(trajs), "w")
            json.dump(joint_state_data, out_file1)
            #print(joint_state_data)
            out_file1.close()
            out_file2 = open("/home/robot/DMP_data_EE_pick_{}.json".format(trajs), "w")
            json.dump(EE_pose_data, out_file2)
            out_file2.close()
            p = 1
            joint_state_data = []
            EE_pose_data = []
        
    if key_pressed == 'q' and p == 1:
            print('Place Trj Collected')
            #print(joint_state_data)
            out_file1.close()
            out_file2 = open("/home/robot/DMP/DMP_data_EE_place_{}.json".format(trajs), "w")
            json.dump(EE_pose_data, out_file2)
            out_file2.close()
'''

'''
Sends velocity commands ~200 Hz
Reads and stores pos, vel, torque (effort) at ~400 Hz
'''

class DataRecorder(object):

    def __init__(self):
        rospy.Subscriber('/wam/joint_states', JointState, self.cb_joint_state)
        rospy.Subscriber('/wam/pose', PoseStamped, self.cb_ee_pose)
        rospy.Subscriber('keyboard_command', String, self.cb_keyboard)
        self.close_grasp = rospy.ServiceProxy('/zeus/bhand/open_grasp',Empty) 
        self.open_grasp = rospy.ServiceProxy('/zeus/bhand/close_grasp',Empty)
        self.joint = rospy.ServiceProxy('/wam/hold_joint_pos',Hold) 
        

        self.joint_state_data = []
        self.ee_pose_data = []
        self.picked = False
        self.collect = False
    
    def cb_joint_state(self, data : JointState):
        # print ("joint kardam")
        joint_state = {'time': data.header.stamp.secs+data.header.stamp.nsecs*10^(-9),
                'position' : data.position,
                'velocity' : data.velocity,
                'effort' : data.effort}
        if self.collect:
            #print('joint kardam')
            self.joint_state_data.append(joint_state)
    
    def cb_ee_pose(self, data : PoseStamped):
        # print ("pose kardam")
        EE_pose = {'position' : (data.pose.position.x, data.pose.position.y, data.pose.position.z),
                #'orientation' : data.pose.orientation}
                'orientation': (data.pose.orientation.x, data.pose.orientation.y, data.pose.orientation.z,  data.pose.orientation.w)}
        if self.collect:
            #print('EE kardam')
            self.ee_pose_data.append(EE_pose)

    def cb_keyboard(self, key_pressed : String):
        #print ("hhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhh {}".format(key_pressed))
        pick_id = len(glob.glob('/home/robot/DMP/DMP_data_joint_pick_*.json'))
        place_id = len(glob.glob('/home/robot/DMP/DMP_data_joint_place_*.json'))
        #print("{} trajectories exist".format(pick_id))
        if key_pressed.data == 's' and self.picked == False:
            print('Data collecting started')
            self.collect = True

        if key_pressed.data == 'p' and self.picked == False:  # if key 'p' is pressed 
                self.open_grasp()
                self. joint(False) #To be able to move the arm
                print('Pick Trj Collected')
                out_file1 = open("/home/robot/DMP/DMP_data_joint_pick_{}.json".format(pick_id), "w")
                json.dump(self.joint_state_data, out_file1)
                #print(joint_state_data)
                out_file1.close()
                out_file2 = open("/home/robot/DMP/DMP_data_EE_pick_{}.json".format(pick_id), "w")
                json.dump(self.ee_pose_data, out_file2)
                out_file2.close()
                self.picked = True
                self.joint_state_data = []
                self.ee_pose_data = []
            
        if key_pressed.data == 'q' and self.picked:
                self.close_grasp()
                self. joint(False) #To be able to move the arm
                print('Place Trj Collected')
                #print(joint_state_data)
                out_file1 = open("/home/robot/DMP/DMP_data_joint_place_{}.json".format(place_id), "w")
                json.dump(self.joint_state_data, out_file1)
                out_file1.close()
                out_file2 = open("/home/robot/DMP/DMP_data_EE_place_{}.json".format(place_id), "w")
                json.dump(self.ee_pose_data, out_file2)
                out_file2.close()
                self.joint_state_data = []
                self.ee_pose_data = []
                self.picked = False
                self.collect = False
                
        if key_pressed.data == 'g':
            go_ready_pos()
            self. joint(False) #To be able to move the arm
        


if __name__ == '__main__':
    rospy.init_node("DMP_node")
    # Create a ROS publisher
    jnt_vel_pub = rospy.Publisher('/wam/jnt_vel_cmd', RTJointVel, queue_size=1)
    jnt_pos_pub = rospy.Publisher('/wam/jnt_pos_cmd', RTJointPos, queue_size=1)

    # init_joint_states_listener()
    # init_EE_pose_listener()
    # init_key_pose_listener()    
    go_ready_pos()
	
    joint = rospy.ServiceProxy('/wam/hold_joint_pos',Hold) 
    joint(False) #To be able to move the arm

    recorder = DataRecorder()

     
    rospy.spin()
    """
    while not rospy.is_shutdown():
        #joint_state_data.append(joint_state_final)
        #EE_pose_data.append(EE_pose_final)	
        print (i)
        i = i+1
        events = pygame.event.get()
        for event in events:
            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_LEFT:
                    print('Pick Trj Collected')
                    out_file1 = open("/home/robot/DMP/DMP_data_joint_pick_{}.json".format(trajs), "w")
                    json.dump(joint_stat            chmod +xe_data, out_file1)
						#print(joint_state_data)
                    out_file1.close()
                    out_file2 = open("/home/robot/DMP/DMP_data_EE_pick_{}.json".format(trajs), "w")
                    json.dump(EE_pose_data, out_file2)
                    out_file2.ckey_pressedlose()
                    p = 1

                if event.key == pygame.K_RIGHT and p == 1:
                    json.dump(joint_state_data, out_file1)
					#print(joint_state_data)
                    out_file1.close()
                    out_file2 = open("/home/robot/DMP/DMP_data_EE_place_{}.json".format(trajs), "w")
                    json.dump(Ekey_pressedE_pose_data, out_file2)
                    out_file2.close()

                    
        if key_pressed == 'p':  # if key 'q' is pressed 
            print('Pick Trj Collected')
            out_file1 = open("/home/robot/DMP_data_joint_pick_{}.json".format(trajs), "w")
            json.dump(joint_state_data, out_file1)
            #print(joint_state_key_pressedata, out_file2)
            out_file2.close()
            p = 1/zeus/bhand/initialize
            
        #rate.sleep()   
    """	  

    
    
