#!/usr/bin/env python2
from __future__ import division, print_function

import time
import subprocess

import numpy as np
import cv2
import zmq
import rospy
from cv_bridge import CvBridge, CvBridgeError

from geometry_msgs.msg import Point, Vector3, Vector3Stamped, Quaternion
from std_msgs.msg import Header
import baxter_interface
import arm_scenario_simulator as arm_sim
from arm_scenario_experiments import Recorder, utils, baxter_utils

from sensor_msgs.msg import Image

SERVER_PORT = 7777
REF_POINT = [0.6, 0.30, 0.20]
IK_SEED_POSITIONS = [-1.535, 1.491, -0.038, 0.194, 1.546, 1.497, -0.520]

bridge = CvBridge()


class ImageCallback(object):
    def __init__(self):
        super(ImageCallback, self).__init__()
        self.last_good_img = None

    def image_callback(self, msg):
        try:
            # Convert your ROS Image message to OpenCV
            cv2_img = bridge.imgmsg_to_cv2(msg, "bgr8")
            self.last_good_img = cv2_img
        except CvBridgeError, e:
            print("CvBridgeError:", e)

def getActions(delta_pos, n_actions):
    """
    Get list of possible actions
    :param delta_pos: (float)
    :param n_actions: (int)
    :return: (numpy matrix)
    """
    possible_deltas = [i * delta_pos for i in range(-1, 2)]
    actions = []
    for dx in possible_deltas:
        for dy in possible_deltas:
            for dz in possible_deltas:
                if dx == 0 and dy == 0 and dz == 0:
                    continue
                # Allow only move in one direction
                if abs(dx) + abs(dy) + abs(dz) > delta_pos:
                    continue
                actions.append([dx, dy, dz])

    assert len(actions) == n_actions, "Wrong number of actions: {}".format(len(actions))

    # action_to_idx = {action: idx for idx, action in enumerate(actions)}
    # # Sort the dictionnary to have a consistent order
    # action_to_idx = OrderedDict(sorted(action_to_idx.items(), key=lambda item: item[1]))

    return np.array(actions)

def randomAction(possible_actions):
    action_idx = np.random.randint(len(possible_actions))
    return possible_actions[action_idx]

def move_left_arm_to_init():
    """
    Initialize robot left arm to starting position (hardcoded)
    :return: ([float])
    """
    joints = None
    while not joints:
        position = REF_POINT
        try:
            joints = baxter_utils.IK(left_arm, position, ee_orientation, IK_SEED_POSITIONS)
        except Exception:
            try:
                joints = baxter_utils.IK(left_arm, position, ee_orientation, IK_SEED_POSITIONS)
            except Exception:
                raise
    left_arm.move_to_joint_positions(joints)
    return position

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

rospy.init_node('gym_gazebo_server', anonymous=True)

image_topic = "/cameras/head_camera_2/image"
image_cb = ImageCallback()
# tcp_nodelay=True
img_sub = rospy.Subscriber(image_topic, Image, image_cb.image_callback)


left_arm = baxter_interface.Limb('left')
right_arm = baxter_interface.Limb('right')
ee_orientation = baxter_utils.get_ee_orientation(left_arm)
ee_orientation_right = baxter_utils.get_ee_orientation(right_arm)

# those are publishers specially made to be listenned by the recorder,
# so that we can publish and thus record exactly what we want when we want
action_pub = rospy.Publisher('/robot/limb/left/endpoint_action', Vector3Stamped, queue_size=1)
button_pos_pub = rospy.Publisher('/button1/position', Point, queue_size=1)


lever = arm_sim.Lever('lever1')
button = arm_sim.Button('button1')
button_orientation = button.get_state().pose.orientation
baxter = arm_sim.Button('baxter')
baxter_pose = baxter.get_state().pose
baxter_position = utils.point2array(baxter_pose.position)
baxter_orientation = utils.quat2array(baxter_pose.orientation)

# ===== Actions ====
DELTA_POS = 0.05
possible_actions = getActions(DELTA_POS, n_actions=6)

rospy.sleep(1)
print("Initializing robot...")
# Init robot pose
subprocess.call(["rosrun", "arm_scenario_experiments", "button_init_pose"])
print("Init Robot pose over")
# end_point_position = baxter_utils.get_ee_position(left_arm)
end_point_position = move_left_arm_to_init()

print('Starting up on port number {}'.format(SERVER_PORT))
context = zmq.Context()
socket = context.socket(zmq.PAIR)

socket.bind("tcp://*:%s" % SERVER_PORT)


print("Waiting for client...")
socket.send_json({'msg': 'hello'})
print("Connected to client")

action = [0, 0, 0]
joints = None

while True:
    msg = socket.recv_json()
    command = msg.get('command', '')
    if command == 'reset':
        print('Reset')

    elif command == 'action':
        action = np.array(msg['action'])
        print("action:", action)

    elif command == "exit":
        break
    else:
        raise ValueError("Unknown command: {}".format(msg))

    # action = randomAction(possible_actions)
    end_point_position_candidate = end_point_position + action

    print("Position:", end_point_position_candidate)
    try:
        joints = baxter_utils.IK(left_arm, end_point_position_candidate, ee_orientation)
    except:
        print("[ERROR] no joints position returned by the Inverse Kinematic fn")
        print("end_point_position_candidate:{}".format(end_point_position_candidate))

    if joints:
        action_pub.publish(Vector3Stamped(Header(stamp=rospy.Time.now()), Vector3(*action)))
        end_point_position = end_point_position_candidate
        left_arm.move_to_joint_positions(joints, timeout=3)

    # Send world position data, etc
    socket.send_json(
        {
            # XYZ position
            "position": list(end_point_position),
            "reward": int(button.is_pressed()),
        },
        flags=zmq.SNDMORE
    )

    img = image_cb.last_good_img
    # to contiguous, otherwise ZMQ will complain
    img = np.ascontiguousarray(img, dtype=np.uint8)
    sendMatrix(socket, img)

print("Server Exiting...")

socket.close()
