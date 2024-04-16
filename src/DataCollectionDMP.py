#!/usr/bin/python3

import numpy as np
import rospy 
import h5py
import time
import numpy

from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from wam_srvs.srv import JointMove
from wam_srvs.srv import Hold
from std_srvs.srv import Empty

import json
import glob

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


class DataRecorder(object):

    def __init__(self, save_file):
        rospy.Subscriber('/wam/joint_states', JointState, self.cb_joint_state)
        rospy.Subscriber('/wam/pose', PoseStamped, self.cb_ee_pose)
        rospy.Subscriber('keyboard_command', String, self.cb_keyboard)
        self.open_grasp = rospy.ServiceProxy('/zeus/bhand/open_grasp', Empty) 
        self.close_grasp = rospy.ServiceProxy('/zeus/bhand/close_grasp', Empty)
        self.save_file = save_file

        self.hd5f_file = h5py.File(self.save_file, 'a')

        self.joint_state_data = []
        self.ee_pose_data = []
        self.collect = False

    def reset(self):
        self.joint_state_data = [1]
        self.ee_pose_data = [0]
        self.collect = False

    def get_next_group_id(self):
        existing_groups = self.hd5f_file.keys()
        if len(existing_groups) == 0:
            return 0
        else:
            return max([int(g) for g in existing_groups]) + 1
    
    def cb_joint_state(self, data : JointState):
        joint_state = {'time': data.header.stamp.secs+data.header.stamp.nsecs*10^(-9),
                'position' : data.position,
                'velocity' : data.velocity,
                'effort' : data.effort}
        if self.collect:
            self.joint_state_data.append(joint_state)
    
    def cb_ee_pose(self, data : PoseStamped):
        EE_pose = {'position' : (data.pose.position.x, data.pose.position.y, data.pose.position.z),
                'orientation': (data.pose.orientation.x, data.pose.orientation.y, data.pose.orientation.z,  data.pose.orientation.w)}
        if self.collect:
            self.ee_pose_data.append(EE_pose)

    def cb_keyboard(self, key_pressed : String):

        # Begin data collection
        if key_pressed.data == 'b':
            print('Data collecting started')
            self.collect = True
        
        # End data collection
        elif key_pressed.data == 'e':
            print('Data collecting ended')
            group_id = self.get_next_group_id()
            print('Group ID:', group_id)
            traj_group = self.hd5f_file.create_group(str(group_id))

            traj_group.attrs['creation_time'] = time.strftime('%Y-%m-%d %H:%M:%S', time.localtime())
            # traj_group.create_dataset('joint_state', data=self.joint_state_data)
            # traj_group.create_dataset('joint_state', data=self.joint_state_data)
            traj_group.create_dataset('joint_state', data=range(10))
            traj_group.create_dataset('ee_pose', data=range(10))
            self.reset()

        elif key_pressed.data == 'o':
            self.open_grasp()
            
        elif key_pressed.data == 'c':
            self.close_grasp()
        else:
            print('Invalid key:', key_pressed.data)

    def __del__(self):
        self.hd5f_file.close()

if __name__ == '__main__':
    rospy.init_node("teleop_collection_node")
	
    recorder = DataRecorder('test.h5')
     
    rospy.spin()

