#!/usr/bin/python3

import sys
import os
import threading
import numpy as np
import rospy 
import h5py
import time
import cv2
import argparse
from pathlib import Path

from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
from wam_srvs.srv import JointMove
from std_srvs.srv import Empty
from bhand_teleop_msgs.msg import BhandTeleop

class VideoCatureThread(threading.Thread):
    def __init__(self, callback, cap):
        threading.Thread.__init__(self)
        self.callback = callback
        self.running = False
        self.cap = cap

    def run(self):
        self.running = True
        while self.running:
            ret, frame = self.cap.read()
            if ret:
                self.callback(frame)
            else:
                print("Error: Could not read frame")
                
        self.cap.release()

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
            if len(data.velocity) == 0:
                data.velocity = [np.nan]*len(data.position)
            if len(data.effort) == 0:
                data.effort = [np.nan]*len(data.position)
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

class HandVelCmdData(DataInterface):
    def __init__(self, topic_name):
        super().__init__()
        self.cmd_vel_data = []
        self.time = []
        rospy.Subscriber(topic_name, BhandTeleop, self.add)

    def add(self, data):
        if self.check_collect_fn and self.check_collect_fn():
            cmd_vel = np.array([data.spread, data.grasp], dtype=float)
            self.cmd_vel_data.append(cmd_vel)
            self.time.append(rospy.get_time() - self.start_time_fn())

    def reset(self):
        self.cmd_vel_data = []
        self.time = []

    def add_to_hdf5(self, hdf5_handle):
        cmd_vel_data = np.array(self.cmd_vel_data)
        joint_dset = hdf5_handle.create_dataset('cmd_vel', data=cmd_vel_data)
        joint_dset.attrs['description'] = "size = (n,2) where 3 is (spread, grasp)" 
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
    def __init__(self, cap):
        super().__init__()
        self.video = []
        self.time = []
        
        self.video_thread = VideoCatureThread(self.add, cap)
        self.video_thread.start()

    def add(self, data):
        if self.check_collect_fn and self.check_collect_fn():
            self.video.append(data)
            self.time.append(rospy.get_time() - self.start_time_fn())

    def reset(self):
        self.video = []
        self.time = []

    def add_to_hdf5(self, hdf5_handle):
        frame_paths = []
        parent_path = Path(hdf5_handle.file.filename).parent
        dir =  parent_path / hdf5_handle.name[1:]
        dir.mkdir(parents=True, exist_ok=True)
        rel_path = dir.relative_to(parent_path)
        for i, frame in enumerate(self.video):
            frame_filename = f"{i}.png"
            frame_path_str = str(dir / frame_filename)
            frame_relpath_str = str(rel_path / frame_filename)
            print(frame_path_str)
            frame_paths.append(frame_relpath_str)
            cv2.imwrite(frame_path_str, frame, [cv2.IMWRITE_PNG_COMPRESSION, 8])

        string_dt = h5py.string_dtype(encoding="utf-8")
        path_dst = hdf5_handle.create_dataset('frame_paths', (len(frame_paths),), dtype=string_dt)
        path_dst[:] = frame_paths
        time_data = np.array(self.time)
        time_dset = hdf5_handle.create_dataset('time', data=time_data)

    def cleanup(self):
        self.video_thread.stop()
            

class DataRecorder(object):

    def __init__(self, save_path):
        self.open_grasp = rospy.ServiceProxy('/zeus/bhand/open_grasp', Empty) 
        self.close_grasp = rospy.ServiceProxy('/zeus/bhand/close_grasp', Empty)
        self.save_file = save_path

        dest_path = Path(save_path)
        dest_path.mkdir(parents=True, exist_ok=True)

        self.hd5f_file = h5py.File(str(dest_path / "data.h5"), 'a')

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
            print("saved")
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
    parser = argparse.ArgumentParser()
    parser.add_argument("--save_path", type=str, required=True, help="Destination path of data")
    args = parser.parse_args()
    rospy.init_node("teleop_collection_node")

    recorder = DataRecorder(args.save_path)
    
    bhand_joint_state = JointStateData("/bhand/joint_states")
    recorder.add_datatype("bhand/joint_state", bhand_joint_state)

    follower_joint_state = JointStateData("/follower/joint_state")
    recorder.add_datatype("follower/joint_state", follower_joint_state)

    hand_vel_cmd = HandVelCmdData("/bhand_mux/bhand_vel")
    recorder.add_datatype("bhand/vel_cmd", hand_vel_cmd)

    # pipe_line = "aravissrc camera-name=FLIR-0119E8A8 ! video/x-raw, framerate=30/1, format=RGB ! videoconvert ! appsink"
    # cap = cv2.VideoCapture(pipe_line, cv2.CAP_GSTREAMER)
    os.environ["OPENCV_FFMPEG_CAPTURE_OPTIONS"] = "rtsp_transport;udp|fflags;nobuffer|probesize;32|analyzeduration;0|max_delay;1"
    vcap = cv2.VideoCapture("rtsp://192.168.1.11:8554/inhand", cv2.CAP_FFMPEG)
    video_data = VideoData(vcap)
    recorder.add_datatype("video_inhand", video_data)

    recorder.print_help()

    rospy.spin()

