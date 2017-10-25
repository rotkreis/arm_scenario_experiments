import os
from copy import copy

import rospy
import rosbag
import rostopic


class Recorder(object):
    def __init__(self, path, prefix, topics):
        self.path = path
        # TODO: Replace this command with os.makedirs(path, exist_ok=True)
        os.system('mkdir -p ' + path)
        self.prefix = prefix
        self.topics = topics
        self.recorded_topics = {}
        self.publishers = {}
        self.lastMessages = {topic: None for topic in topics}
        self.bag = None

        self._latches = {}
        self._bag_latch = False
        self._not_found_topics = copy(topics)

        self.check_topics()

    def __del__(self):
        self.close_bag()

    def new_bag(self, name):
        if self.bag:
            raise Exception('First close the current bag before creating a new one')
        bag_name = "{}/{}.bag".format(self.path, name)
        rospy.loginfo('Creating new bag : ' + bag_name)
        self.bag = rosbag.Bag(bag_name, 'w')

    def close_bag(self):
        if self.bag:
            print('Closing the bag')
            while self._bag_latch: pass
            self.bag.close()
            self.bag = None
        else:
            rospy.loginfo('No bag currently open')

    def new_topic_name(self, topic):
        prefix = self.prefix
        if topic[0] != '/' and prefix[-1] != '/': prefix += '/'
        return prefix + topic

    """ Callback for recording signals"""
    def make_callback(self, topic):
        written_topic = self.recorded_topics[topic]

        def callback(message):
            while self._latches[topic]:
                pass
            self._latches[topic] = True
            self.lastMessages[topic] = message
            self._latches[topic] = False

        return callback

    def check_topic(self, topic):
        message_class = rostopic.get_topic_class(topic)[0]  # None if topic is not already published
        if message_class:
            self.recorded_topics[topic] = self.new_topic_name(topic)
            self._latches[topic] = False
            rospy.Subscriber(topic, message_class, self.make_callback(topic), queue_size=10)
            self.publishers[topic] = rospy.Publisher(self.recorded_topics[topic], message_class, queue_size=1)
            rospy.loginfo('create suscriber for ' + topic)
            return True
        return False

    def check_topics(self):
        self._not_found_topics = [topic for topic in self._not_found_topics if not self.check_topic(topic)]

    def all_buffers_full(self, excepts=None):
        excepts = excepts if excepts is not None else []
        for topic, msg in self.lastMessages.iteritems():
            if topic not in excepts and not msg:
                print("No msg for topic {}".format(topic))
                return False
        return True

    def dump(self, topic):
        if not self.bag:
            raise Exception('Cannot dump ' + topic + ' : no bag opened')
        if isinstance(topic, list):
            return [self.dump(t) for t in topic]
        if topic not in self.recorded_topics:
            self.check_topic(topic)
            rospy.loginfo('Checking for ' + topic + ' again')
            return
        if not self.lastMessages[topic]:
            return
        while self._latches[topic]:
            pass
        self._latches[topic] = True
        self._bag_latch = True
        message = self.lastMessages[topic]
        try:
            self.bag.write(self.recorded_topics[topic], message)
            self.publishers[topic].publish(message)
            self.lastMessages[topic] = None
            return message
        finally:
            self._latches[topic] = False
            self._bag_latch = False

    def dump_all(self):
        return self.dump(self.topics)
