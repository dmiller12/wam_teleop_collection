#!/usr/bin/python3

import sys
import threading
import numpy as np
import rospy 
import h5py
import time
import cv2

from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
from wam_srvs.srv import JointMove
from std_srvs.srv import Empty

class VideoCatureThread(threading.Thread):
    def __init__(self, callback):
        threading.Thread.__init__(self)
        self.callback = callback
        self.running = False

    def run(self):
        self.running = True
        # pipe_line = "aravissrc camera-name=FLIR-0119E8A8 ! video/x-raw, format=RGB ! videoconvert ! appsink"
        pipe_line = "videotestsrc ! appsink"
        cap = cv2.VideoCapture(pipe_line, cv2.CAP_GSTREAMER)
        # if not cap.isOpened():
        #     print("Error: Could not open video stream")
        #     return
        while self.running:
            ret, frame = cap.read()
            if ret:
                self.callback(frame)
            else:
                print("Error: Could not read frame")
                
        cap.release()

    def stop(self):
        self.running = False
            

class DataRecorder(object):

    def __init__(self, save_file):
        rospy.Subscriber('/wam/joint_states', JointState, self.cb_joint_state)
        rospy.Subscriber('/wam/pose', PoseStamped, self.cb_ee_pose)
        # rospy.Subscriber('keyboard_command', String, self.cb_keyboard)
        self.open_grasp = rospy.ServiceProxy('/zeus/bhand/open_grasp', Empty) 
        self.close_grasp = rospy.ServiceProxy('/zeus/bhand/close_grasp', Empty)
        self.save_file = save_file

        self.hd5f_file = h5py.File(self.save_file, 'a')

        self.joint_state_data = []
        self.ee_pose_data = []
        self.joint_state_time = []
        self.video = []
        self.collect = False

        self.video_thread = VideoCatureThread(self.cb_video_capture)
        self.video_thread.start()

        self.stdin_thread = threading.Thread(target=stdin_capture, args=(self.cb_keyboard,))
        self.stdin_thread.daemon = True
        self.stdin_thread.start()

    def reset(self):
        self.joint_state_data = []
        self.ee_pose_data = []
        self.joint_state_time = []
        self.video = []
        self.collect = False

    def get_next_group_id(self):
        existing_groups = self.hd5f_file.keys()
        group_len = len(existing_groups)
        return group_len 

    def cb_joint_state(self, data : JointState):
        if self.collect:
            joint_state = np.array([data.position, data.velocity, data.effort], dtype=float)
            self.joint_state_time.append(data.header.stamp.nsecs)
            self.joint_state_data.append(joint_state)
    
    def cb_ee_pose(self, data : PoseStamped):
        if self.collect:
            pose = np.array([data.pose.position.x, data.pose.position.y, data.pose.position.z, data.pose.orientation.x, data.pose.orientation.y, data.pose.orientation.z, data.pose.orientation.w])
            self.ee_pose_data.append(pose)

    def cb_video_capture(self, data):
        if self.collect:
            self.video.append(data)

    def cb_keyboard(self, key_pressed):

        # Begin data collection
        if key_pressed == 'b':
            print('\nData collecting started')
            self.collect = True
        
        # End data collection
        elif key_pressed == 'e':
            print('\nData collecting ended')
            group_id = self.get_next_group_id()
            traj_group = self.hd5f_file.create_group(f"traj_{group_id}")

            traj_group.attrs['creation_time'] = time.strftime('%Y-%m-%d %H:%M:%S', time.localtime())
            joint_state_data = np.array(self.joint_state_data)
            joint_dset = traj_group.create_dataset('joint_state', data=joint_state_data)
            joint_dset.attrs['description'] = "size = (n,3,dof) where 3 is (position, velocity, effort)" 
            pose_data = np.array(self.ee_pose_data)
            pose_dset = traj_group.create_dataset('pose', data=pose_data)
            pose_dset.attrs['description'] = "size (n,7) where 7 is (pos.x, pos.y, pos.z, quat.x, quat.y, quat.z, quat.w"

            video_data = np.array(self.video)
            video_dset = traj_group.create_dataset('video', data=video_data)
            self.reset()

        elif key_pressed == 'o':
            self.open_grasp()
            
        elif key_pressed == 'c':
            self.close_grasp()
        elif key_pressed == 'h':
            self.print_help()
        elif key_pressed == 'x':
            self.hd5f_file.close()
            self.video_thread.stop()
            rospy.signal_shutdown('Exiting...')

            sys.exit(0)
        else:
            print('Invalid key:', key_pressed)

    def print_help(self):
        print('h: Print help')
        print('b: Begin data collection')
        print('e: End data collection')
        print('o: Open grasp')
        print('c: Close grasp')
        print('x: Exit')
    def __del__(self):
        self.hd5f_file.close()

def stdin_capture(callback):
    while True:
        print("Enter a key: ")
        input_char = sys.stdin.readline().rstrip('\n')[0]
        callback(input_char)

if __name__ == '__main__':
    rospy.init_node("teleop_collection_node")
	
    recorder = DataRecorder('test.h5')
    recorder.print_help()

    rospy.spin()
