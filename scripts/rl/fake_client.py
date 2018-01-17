#!/usr/bin/env python
from __future__ import division, print_function

import time

import numpy as np
import zmq

import cv2

# For Python 3 compatibility
import sys
if sys.version_info > (3,):
    buffer = memoryview


def recvMatrix(socket):
    """Receive a numpy array over zmq"""
    metadata = socket.recv_json()
    msg = socket.recv(copy=True, track=False)
    buf = buffer(msg)
    A = np.frombuffer(buf, dtype=metadata['dtype'])
    return A.reshape(metadata['shape'])

# Port to connect to on the server
SERVER_PORT = 7777
HOSTNAME = 'localhost'


# Connect to the Gym bridge ROS node
context = zmq.Context()
socket = context.socket(zmq.PAIR)
socket.connect("tcp://%s:%s" % (HOSTNAME, SERVER_PORT))

print("Waiting for server...")
msg = socket.recv_json()
print("Connected to server")

# Tell the server to reset the simulation

for _ in range(5):
    socket.send_json({ "command": "action"})
    # Receive state data (position, etc)
    state_data = socket.recv_json()
    print(state_data)

    # Receive a camera image from the server
    img = recvMatrix(socket)

    # cv2.imshow("Image", img)
    # key = cv2.waitKey(0) & 0xff


    # time.sleep(0.5)

socket.send_json({ "command": "exit"})
cv2.destroyAllWindows()

print("Client exiting...")

# # Send the action to the server
# socket.send_json({
#     "command":"action",
#     "values": [ float(action[0]), float(action[1]) ]
# })

socket.close()
