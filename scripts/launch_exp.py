#!/usr/bin/env python

import roslaunch
import rospy
import time
import os

import baxter_interface
from baxter_interface import CHECK_VERSION

sub_folder = '_'.join(time.ctime().split())
data_folder = '/home/masson/Documents/DataSets/dataBaxter/'+sub_folder
os.mkdir(data_folder)

recording_rate = 10
cameras_fps = 50
cameras_resolution = (320,200)

cameras = ['/cameras/head_camera/image/compressed', '/cameras/right_hand_camera/image/compressed']
topics_to_record = cameras + ['/robot/joint_states']
topics_arg = ' '.join(topics_to_record)


try:
    rospy.wait_for_service('/cameras/list', timeout=1) # test if the service is available
    _REAL_ROBOT = True
    print("You're using the robot")
    rospy.loginfo("You're using the robot")
except rospy.ROSException: # if not, it is probably cause we're using the simulator
    _REAL_ROBOT = False
    print("You're using the simulator")
    rospy.loginfo("You're using the simulator")

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

    babbler = launch.launch(roslaunch.core.Node('ann4smc', 'atomic_babbler.py', args=('-ph' if _REAL_ROBOT else '') ))
    recorder = launch.launch(roslaunch.core.Node('ann4smc', 'record_state.py', args='-r '+str(recording_rate)+' -p '+data_folder+' -t '+topics_arg))

    while(babbler.is_alive()): time.sleep(1)

    try: recorder.stop()
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
