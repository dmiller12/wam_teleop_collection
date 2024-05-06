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
        pipe_line = "aravissrc camera-name=FLIR-0119E8A8 ! video/x-raw, framerate=30/1, format=RGB ! videoconvert ! appsink"
        cap = cv2.VideoCapture(pipe_line, cv2.CAP_GSTREAMER)
        while self.running:
            ret, frame = cap.read()
            if ret:
                self.callback(frame)
            else:
                print("Error: Could not read frame")
                
        cap.release()

    def stop(self):
        self.running = False

class DataInterface:
    def __init__(self):
        self.check_collection_fn = None

    def add(self, data):
        raise NotImplementedError

    def reset(self):
        raise NotImplementedError

    def add_to_hdf5(self, hdf5_handle):
        raise NotImplementedError

    def set_check_collect_fn(self, check_collect_fn):
        self.check_collect_fn = check_collect_fn

    def set_start_time_fn(self, time_fn):
        self.start_time_fn = time_fn

    def cleanup(self):
        pass

class JointStateData(DataInterface):
    def __init__(self, topic_name):
        super().__init__()
        self.joint_state_data = []
        self.time = []
        rospy.Subscriber(topic_name, JointState, self.add)

    def add(self, data):
        if self.check_collect_fn and self.check_collect_fn():
            joint_state = np.array([data.position, data.velocity, data.effort], dtype=float)
            self.joint_state_data.append(joint_state)
            self.time.append(rospy.get_time() - self.start_time_fn())

    def reset(self):
        self.joint_state_data = []
        self.time = []

    def add_to_hdf5(self, hdf5_handle):
        joint_state_data = np.array(self.joint_state_data)
        joint_dset = hdf5_handle.create_dataset('joint_state', data=joint_state_data)
        joint_dset.attrs['description'] = "size = (n,3,dof) where 3 is (position, velocity, effort)" 
        time_data = np.array(self.time)
        time_dset = hdf5_handle.create_dataset('time', data=time_data)

class PoseData(DataInterface):
    def __init__(self, topic_name='/wam/pose'):
        super().__init__()
        self.ee_pose_data = []
        self.time = []
        rospy.Subscriber(topic_name, PoseStamped, self.add)
        self.collect = False

    def add(self, data):
        if self.check_collect_fn and self.check_collect_fn():
            pose = np.array([data.pose.position.x, data.pose.position.y, data.pose.position.z, data.pose.orientation.x, data.pose.orientation.y, data.pose.orientation.z, data.pose.orientation.w])
            self.ee_pose_data.append(pose)
            self.time.append(rospy.get_time() - self.start_time_fn())

    def reset(self):
        self.ee_pose_data = []
        self.time = []
    
    def add_to_hdf5(self, hdf5_handle):
        pose_data = np.array(self.ee_pose_data)
        pose_dset = hdf5_handle.create_dataset('pose', data=pose_data)
        pose_dset.attrs['description'] = "size (n,7) where 7 is (pos.x, pos.y, pos.z, quat.x, quat.y, quat.z, quat.w)"
        time_data = np.array(self.time)
        time_dset = hdf5_handle.create_dataset('time', data=time_data)

class VideoData(DataInterface):
    def __init__(self):
        super().__init__()
        self.video = []
        self.time = []
        self.video_thread = VideoCatureThread(self.add)
        self.video_thread.start()

    def add(self, data):
        if self.check_collect_fn and self.check_collect_fn():
            self.video.append(data)
            self.time.append(rospy.get_time() - self.start_time_fn())

    def reset(self):
        self.video = []
        self.time = []

    def add_to_hdf5(self, hdf5_handle):
        video_data = np.array(self.video)
        video_dset = hdf5_handle.create_dataset('video', data=video_data, compression="gzip")
        time_data = np.array(self.time)
        time_dset = hdf5_handle.create_dataset('time', data=time_data)

    def cleanup(self):
        self.video_thread.stop()
            

class DataRecorder(object):

    def __init__(self, save_file):
        self.open_grasp = rospy.ServiceProxy('/zeus/bhand/open_grasp', Empty) 
        self.close_grasp = rospy.ServiceProxy('/zeus/bhand/close_grasp', Empty)
        self.save_file = save_file

        self.hd5f_file = h5py.File(self.save_file, 'a')

        self.collect = False

        self.stdin_thread = threading.Thread(target=stdin_capture, args=(self.cb_keyboard,))
        self.stdin_thread.daemon = True
        self.stdin_thread.start()
        self.datatypes = {}

    def add_datatype(self, name: str, data_type: DataInterface):
        if name not in self.datatypes:
            self.datatypes[name] = data_type
            self.datatypes[name].set_start_time_fn(self.get_start_time)
            self.datatypes[name].set_check_collect_fn(self.should_collect)
        else:
            raise ValueError(f'Data type {name} already exists')

    def reset(self):
        self.collect = False
        for name, data_type in self.datatypes.items():
            data_type.reset()

    def should_collect(self):
        return self.collect

    def get_start_time(self):
        return self.start_time_seconds

    def get_next_group_id(self):
        existing_groups = self.hd5f_file.keys()
        group_len = len(existing_groups)
        return group_len 

    def cb_keyboard(self, key_pressed):

        # Begin data collection
        if key_pressed == 'b':
            group_id = self.get_next_group_id()
            print(f'\nData collecting started for traj {group_id}')
            self.start_time_seconds = rospy.get_time()
            self.collect = True
        
        # End data collection
        elif key_pressed == 'e':
        
            self.collect = False
                
            group_id = self.get_next_group_id()
            print(f'\nData collection ended for traj {group_id}')
            print("saving...")
            traj_group = self.hd5f_file.create_group(f"traj_{group_id}")

            traj_group.attrs['creation_time'] = time.strftime('%Y-%m-%d %H:%M:%S', time.localtime())
            for name, data_type in self.datatypes.items():
                data_group = traj_group.create_group(name)
                data_type.add_to_hdf5(data_group)
            self.reset()
        elif key_pressed == 'o':
            self.open_grasp()
            
        elif key_pressed == 'c':
            self.close_grasp()
        elif key_pressed == 'h':
            self.print_help()
        elif key_pressed == 'x':
            self.hd5f_file.close()

            for name, data_type in self.datatypes.items():
                data_type.cleanup()

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
    
    leader_joint_state = JointStateData("leader/wam/joint_state")
    recorder.add_datatype("leader/joint_state", leader_joint_state)

    leader_joint_pose = PoseData("leader/wam/pose")
    recorder.add_datatype("leader/pose", leader_joint_pose)

    follower_joint_state = JointStateData("follower/wam/joint_state")
    recorder.add_datatype("follower/joint_state", follower_joint_state)

    follower_joint_pose = PoseData("follower/wam/pose")
    recorder.add_datatype("follower/pose", follower_joint_pose)

    video_data = VideoData()
    recorder.add_datatype("video", video_data)

    recorder.print_help()

    rospy.spin()

