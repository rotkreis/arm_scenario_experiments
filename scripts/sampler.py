#!/usr/bin/env python

import rospy
import rostopic
import argparse

from std_msgs.msg import Header
from sensor_msgs.msg import (JointState, Image, CompressedImage)

class Sampler(object):
    ''' Republish the joints states, head camera images and right hand camera image at a given rate.

    The rate MUST be specified and be an integer.
    '''

    def __init__(self, rate, topics):
        self.rate = rospy.Rate(rate)

        self.topics = []
        self.publishers = {}
        self.lastReceivedMessage = {}

        def make_callback(topic):
            def callback(message):
                self.lastReceivedMessage[topic] = message
            return callback

        for topic in topics:
            if topic in self.topics: continue
            self.topics.append(topic)
            message_class = rostopic.get_topic_class(topic)[0]
            callback = make_callback(topic)
            rospy.Subscriber(topic, message_class, callback, queue_size = 1)
            new_topic_name = '/sampled_r'+str(rate)
            if topic[0] is not '/': new_topic_name += '/'
            new_topic_name += topic
            publisher = rospy.Publisher(new_topic_name, message_class, queue_size=1)
            print('Created topic '+new_topic_name)
            self.publishers[topic] = publisher
            self.lastReceivedMessage[topic] = None

    def ok(self):
        return self.on and (not rospy.is_shutdown())

    def start(self):
        self.on = True
        self.publish()

    def stop(self):
        self.on = False

    def publish(self):
        while self.ok():
            self.rate.sleep()
            for topic in self.topics:
                if self.lastReceivedMessage[topic]: self.publishers[topic].publish(self.lastReceivedMessage[topic])
        self.on = False

if __name__ == '__main__':
    parser = argparse.ArgumentParser(formatter_class=argparse.RawDescriptionHelpFormatter, description=Sampler.__doc__)
    parser.add_argument( '-r', '--rate', type=int, required=True, help="the sampling rate")
    parser.add_argument( '-t', '--topic', type=str, required=True, nargs='+' , help="topics to be resampled")
    args = parser.parse_args(rospy.myargv()[1:])

    print("Initializing node... ")
    rospy.init_node('Republisher', anonymous=True)
    print("Republishing. Press Ctrl-C to stop.")
    Sampler(args.rate, args.topic).start()
