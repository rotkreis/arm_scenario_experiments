#!/usr/bin/env python

import argparse

import rospy
import rosbag
import rostopic

from std_msgs.msg import (Header, String, Empty)
import compressed_image_transport
from sensor_msgs.msg import (JointState, Image, CompressedImage)

class Recorder(object):
    ''' Republish the joints states, head camera images and right hand camera image at a given rate.

    The rate MUST be specified and be an integer.
    '''

    def __init__(self, path, rate, topics):
        self.path = path
        self.rate = rospy.Rate(rate)
        self.on = False
        self.topics = {}
        self.lastMessages = {}
        self.current_pose_counter = 0

        ''' Callbacks for interaction with the babbler'''
        def new_pose_callback(message):
            print('New pose')
            self.current_pose_counter +=1
        rospy.Subscriber('/atomic_babbler/status/new_pose', Empty, new_pose_callback)

        def new_record_callback(message):
            joint_name = message.data
            if joint_name=='':
                print('Closing the bag')
                self.on = False
                self.bag.close()
            else:
                bag_name = self.path+'/pose'+str(self.current_pose_counter)+'_'+joint_name+'.bag'
                print('Creating new bag : '+bag_name)
                self.bag = rosbag.Bag(bag_name, 'w')
                self.on = True
        rospy.Subscriber('/atomic_babbler/status/joint_name', String, new_record_callback, queue_size = 1)

        ''' Callback for recording signals'''
        def make_write_callback(topic):
            def callback(message):
                if self.on: self.lastMessages[topic] = message
            return callback

        for topic in topics:
            if topic in self.topics: continue
            written_topic = '/recorded'
            if topic[0]!='/': written_topic+='/'
            written_topic += topic
            self.topics[topic] = written_topic
            self.lastMessages[topic] = None
            message_class = rostopic.get_topic_class(topic)[0]
            callback = make_write_callback(topic)
            rospy.Subscriber(topic, message_class, callback, queue_size = 1)

        while not rospy.is_shutdown():
            self.rate.sleep()
            if self.on:
                for topic, written_topic in self.topics.iteritems():
                    if self.lastMessages[topic]: self.bag.write(written_topic, self.lastMessages[topic])


if __name__ == '__main__':
    try:
        parser = argparse.ArgumentParser(formatter_class=argparse.RawDescriptionHelpFormatter, description=Recorder.__doc__)
        parser.add_argument( '-r', '--rate', type=int, required=True, help="the recording rate")
        parser.add_argument( '-t', '--topic', type=str, required=True, nargs='+' , help="topics to be resampled")
        parser.add_argument( '-p', '--path', type=str, required=True, help="the path to the record file")
        args = parser.parse_args(rospy.myargv()[1:])

        print("Initializing node... ")
        rospy.init_node('state_recorder', anonymous=True)
        print("Recording. Press Ctrl-C to stop.")
        print('rate : ', args.rate)
        print('path : ', args.path)
        print('topics : ', ' '.join(args.topic))
        Recorder(args.path, args.rate, args.topic)
    except Exception:
        pass
