#!/usr/bin/env python

import roslaunch
import rospy
import time
import os
from sensor_msgs.msg import (JointState, Image, CompressedImage)

import baxter_interface
from baxter_interface import CHECK_VERSION
rospy.set_param("~image_transport", "compressed")

sub_folder = time.ctime().replace(' ','_')
data_folder = os.environ['HOME']+'/ros_recorder_'+sub_folder
os.mkdir(data_folder)

recording_rate = 10
cameras_fps = 50
cameras_resolution = (320,200)
cameras = ['/cameras/head_camera_2/image']

try:
    rospy.wait_for_service('/cameras/list', timeout=1) # test if the service is available
    _REAL_ROBOT = True
    print("You're using the robot")
    rospy.loginfo("You're using the robot")
except rospy.ROSException: # if not, it is probably cause we're using the simulator
    _REAL_ROBOT = False
    print("You're using the simulator")
    rospy.loginfo("You're using the simulator")

def republished_name(topic): return topic+"/republished"

if _REAL_ROBOT: topics_to_record = [republished_name(topic)+"/compressed" for topic in cameras]
else : topics_to_record = [topic+"/compressed" for topic in cameras]
topics_to_record += ['/robot/joint_states', '/button1/is_pressed', '/button2/is_pressed', '/button3/is_pressed']
topics_arg = ' '.join(topics_to_record)

def set_cameras():
    if _REAL_ROBOT:
        head_cam = baxter_interface.CameraController('head_camera')
        right_hand_cam = baxter_interface.CameraController('right_hand_camera')
        head_cam.resolution = cameras_resolution
        head_cam.fps = cameras_fps
        right_hand_cam.resolution = cameras_resolution
        right_hand_cam.fps = cameras_fps


def main():
    launch = roslaunch.scriptapi.ROSLaunch()
    launch.start()

    republishers = []
    if  _REAL_ROBOT:
        for topic in cameras:
            republishers.append(launch.launch(roslaunch.core.Node('image_transport', 'republish', args="raw in:="+topic+" out:="+republished_name(topic) )))
    babbler = launch.launch(roslaunch.core.Node('arm_scenario_simulator', 'ik_babbler', args=('-ph' if _REAL_ROBOT else '') ))
    recorder = launch.launch(roslaunch.core.Node('arm_scenario_simulator', 'recorder', args='-r '+str(recording_rate)+' -p '+data_folder+' -t '+topics_arg+' --start'))

    while(not babbler.is_alive()): time.sleep(1)
    print('babbling has started')
    while(babbler.is_alive()): time.sleep(1)
    print('babbling has finished')

    try:
        recorder.stop()
        for rep in republishers: rep.close()
    except Exception: pass
    launch.stop()


if __name__ == '__main__':
    print("Initializing node... ")
    rospy.init_node('Experience_launcher')
    print("Getting robot state... ")
    rs = baxter_interface.RobotEnable(CHECK_VERSION)
    print("Enabling robot... ")
    rs.enable()
    print("Running. Ctrl-c to quit")

    set_cameras()
    main()
