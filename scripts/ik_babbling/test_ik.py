#!/usr/bin/env python

import math
import random
import argparse

import rospy

from tf import transformations as tft
np = tft.numpy

from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    Quaternion,
)
from std_msgs.msg import (Header, String, Empty)

from baxter_core_msgs.srv import (
    SolvePositionIK,
    SolvePositionIKRequest,
)

import baxter_interface
from baxter_interface import Limb, CHECK_VERSION


if __name__ == "__main__":
    parser = argparse.ArgumentParser()

    print("Initializing babbler node... ")
    rospy.init_node("node_clement")
    print("Getting robot state... ")
    rs = baxter_interface.RobotEnable(CHECK_VERSION)
    print("Enabling robot... ")
    rs.enable()
    print("Running. Ctrl-c to quit")

    ns = "ExternalTools/left/PositionKinematicsNode/IKService"
    iksvc = rospy.ServiceProxy(ns, SolvePositionIK)
    ikreq = SolvePositionIKRequest()
    angle = (math.pi/180)*160
    vec = tft.unit_vector([1,-0.1,-0.1])
    pose = PoseStamped(
            header=Header(stamp=rospy.Time.now(), frame_id='base'),
            pose=Pose(
                position=Point(
                    x=0.5,
                    y=0.5,
                    z=0.08,
                ),
                orientation=Quaternion(
                    x=math.sin(angle/2)*vec[0],
                    y=math.sin(angle/2)*vec[1],
                    z=math.sin(angle/2)*vec[2],
                    w=math.cos(angle/2),
                ),
            ),
        )

    ikreq.pose_stamp.append(pose)
    try:
        rospy.wait_for_service(ns, 5.0)
        resp = iksvc(ikreq)
    except (rospy.ServiceException, rospy.ROSException), e:
        rospy.logerr("Service call failed: %s" % (e,))
        raise e

    if (resp.isValid[0]):
        print("SUCCESS - Valid Joint Solution Found :")
        # Format solution into Limb API-compatible dictionary
        joint_solution = dict(zip(resp.joints[0].name, resp.joints[0].position))
        print "\nIK Joint Solution:\n", joint_solution
        print "------------------"
        print "Response Message:\n", resp
    else:
        print("INVALID POSE - No Valid Joint Solution Found.")
        raise Exception

    # set arm joint positions to solution
    arm = baxter_interface.Limb('left')
    while not rospy.is_shutdown():
        arm.set_joint_positions(joint_solution)
        rospy.sleep(0.01)
