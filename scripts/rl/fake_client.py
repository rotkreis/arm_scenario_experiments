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

# Arrow keys
UP_KEY = 82
DOWN_KEY = 84
RIGHT_KEY = 83
LEFT_KEY = 81
ENTER_KEY = 10
SPACE_KEY = 32
EXIT_KEYS = [113, 27]  # Escape and q
D_KEY = 100
U_KEY = 117

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

DELTA_POS = 0.05
times = []
action = [0, 0, 0]
action_dict = {
    LEFT_KEY: [- DELTA_POS, 0, 0],
    RIGHT_KEY: [DELTA_POS, 0, 0],
    UP_KEY: [0, DELTA_POS, 0],
    DOWN_KEY: [0, -DELTA_POS, 0],
    D_KEY: [0, 0, - DELTA_POS],
    U_KEY: [0, 0, DELTA_POS]

}
cv2.imshow("Image", np.zeros((10, 10, 3), dtype=np.uint8))

while True:
    # Retrieve pressed key
    key = cv2.waitKey(0) & 0xff

    if key in EXIT_KEYS:
        break
    elif key in action_dict.keys():
        action = action_dict[key]
    else:
        print("Unknown key: {}".format(key))
        action = [0, 0, 0]

    start_time = time.time()
    socket.send_json({ "command": "action", "action": action})
    # Receive state data (position, etc)
    state_data = socket.recv_json()
    print(state_data)

    # Receive a camera image from the server
    img = recvMatrix(socket)

    cv2.imshow("Image", img)
    # key = cv2.waitKey(0) & 0xff


    times.append(time.time() - start_time)

socket.send_json({ "command": "exit"})
cv2.destroyAllWindows()

print("Client exiting...")

print("{:.2f} FPS".format(len(times) / np.sum(times)))

# # Send the action to the server
# socket.send_json({
#     "command":"action",
#     "values": [ float(action[0]), float(action[1]) ]
# })

socket.close()
