#!/usr/bin/env python2
from __future__ import division, print_function

import time

import numpy as np
import cv2
import zmq
import rospy
from cv_bridge import CvBridge, CvBridgeError

from sensor_msgs.msg import Image

SERVER_PORT = 7777


def sendMatrix(socket, mat):
    """
    Send a numpy mat with metadata over zmq
    :param socket:
    :param mat: (numpy matrix)
    """
    metadata = dict(
        dtype=str(mat.dtype),
        shape=mat.shape,
    )
    # SNDMORE flag specifies this is a multi-part message
    socket.send_json(metadata, flags=zmq.SNDMORE)
    return socket.send(mat, flags=0, copy=True, track=False)


print('Starting up on port number {}'.format(SERVER_PORT))
context = zmq.Context()
socket = context.socket(zmq.PAIR)

socket.bind("tcp://*:%s" % SERVER_PORT)

bridge = CvBridge()


# Number of step messages we've sent to gazebo
num_steps = 0
# Number of images received
num_imgs = 0


class ImageCallback(object):
    def __init__(self):
        super(ImageCallback, self).__init__()
        self.last_good_img = None

    def image_callback(self, msg):
        global num_imgs
        num_imgs += 1

        # print("Received an image")
        # setattr(msg, 'encoding', '')

        try:
            # Convert your ROS Image message to OpenCV
            cv2_img = bridge.imgmsg_to_cv2(msg, "bgr8")
            self.last_good_img = cv2_img
        except CvBridgeError, e:
            print("CvBridgeError:", e)

image_cb = ImageCallback()

rospy.init_node('gym_gazebo_server', anonymous=True)

image_topic = "/cameras/head_camera_2/image"
# tcp_nodelay=True
img_sub = rospy.Subscriber(image_topic, Image, image_cb.image_callback)

print("Waiting for client...")
socket.send_json({'msg': 'hello'})
print("Connected to client")

while True:
    msg = socket.recv_json()
    command = msg.get('command', '')
    if command == 'reset':
        print('Reset')

    elif command == 'action':
        print("action")

    elif command == "exit":
        break
    else:
        raise ValueError("Unknown command: {}".format(msg))

    # Send world position data, etc
    socket.send_json(
        {
            # XYZ position
            "position": [0.0, 0.0, 0.0],
            "reward": 0,
        },
        flags=zmq.SNDMORE
    )

    img = image_cb.last_good_img
    if img is None:
        time.sleep(1)
        img = image_cb.last_good_img

    print(img.shape)
    # to contiguous, otherwise ZMQ will complain
    img = np.ascontiguousarray(img, dtype=np.uint8)
    sendMatrix(socket, img)

print("Server Exiting...")

socket.close()
