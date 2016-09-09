import rospy
import rosbag
from copy import copy
import rostopic

class Recorder(object):
    def __init__(self, path, prefix, topics):
        self.path = path
        self.prefix = prefix
        self.topics= topics
        self.recorded_topics = {}
        self.lastMessages = {}
        self.bag = None

        self._latches = {}
        self._bag_latch = False
        self._not_found_topics = copy(topics)

        self.check_topics()
        for topic in self._not_found_topics:
            rospy.loginfo('WARNING : '+topic+' is not published yet. Use check_topic again later.')


    def new_bag(self, name):
        if self.bag: raise Excpetion('First close the current bag before creating a new one')
        bag_name = self.path+'/'+name+'.bag'
        print('Creating new bag : '+bag_name)
        self.bag = rosbag.Bag(bag_name, 'w')

    def close_bag(self):
        if self.bag:
            print('Closing the bag')
            while self._bag_latch:pass
            self.bag.close()
            self.bag = None
        else:
            rospy.loginfo('No bag currently open')

    def new_topic_name(self, topic):
        prefix= self.prefix
        if topic[0]!='/': prefix+='/'
        return prefix+topic

    ''' Callback for recording signals'''
    def make_callback(self, topic):
        written_topic = self.recorded_topics[topic]
        def callback(message):
            if self.bag:
                while self._latches[topic]: pass
                self._latches[topic] = True
                self.lastMessages[topic] = message
                self._latches[topic] = False
        return callback

    def check_topic(self, topic):
        message_class = rostopic.get_topic_class(topic)[0] # None if topic is not already published
        if message_class: 
            self.recorded_topics[topic] = self.new_topic_name(topic)
            self._latches[topic] = False
            rospy.Subscriber(topic, message_class, self.make_callback(topic), queue_size = 10)
            return True
        return False

    def check_topics(self):   
        self._not_found_topics = [topic for topic in self._not_found_topics if not self.check_topic(topic)]


    def dump(self, topic):
        if isinstance(topic,list): 
            return [self.dump(t) for t in topic]
        if topic not in self.recorded_topics or topic not in self.lastMessages: 
            return
        while self._latches[topic]: pass
        self._latches[topic] = True
        self._bag_latch = True
        message = self.lastMessages[topic]
        try:
            self.bag.write(self.recorded_topics[topic], message)
            return message
        finally:
            self._latches[topic] = False
            self._bag_latch = False    

    def dump_all(self):
        return self.dump(self.recorded_topics.keys())
            

