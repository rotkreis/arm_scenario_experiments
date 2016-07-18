#!/usr/bin/env python

import argparse
import rosbag
import rospy
import rostopic

from sensor_msgs.msg import JointState

parser = argparse.ArgumentParser(formatter_class=argparse.RawDescriptionHelpFormatter)
parser.add_argument('path', type=str, help="the path to the record file")
args = parser.parse_args(rospy.myargv()[1:])

bag = rosbag.Bag(args.path)
for topic, msg, t in bag.read_messages():
    print('message time : ', msg.header.stamp.secs,  format(msg.header.stamp.nsecs, '09d'), topic)
bag.close()
