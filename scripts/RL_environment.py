#!/usr/bin/env python

import rospy

from tf import transformations as tft
np = tft.numpy

from std_msgs.msg import Header
from geometry_msgs.msg import Point, Quaternion, Pose, PoseStamped
import baxter_interface
from baxter_core_msgs.srv import SolvePositionIK, SolvePositionIKRequest
import arm_scenario_simulator as arm_sim
import ann4smc.srv

def point2array(point):
    return np.array([point.x, point.y, point.z])

class Env():
    env = None

    def __init__(self):
        if Env.env is None:
            Env.env = self
            
            self.limb = baxter_interface.Limb('left')
            initial_pose = self.limb.endpoint_pose()
            o = initial_pose['orientation']
            self.ee_orientation = Quaternion(w=o.w, x=o.x, y=o.y, z=o.z)

            self.button = arm_sim.Button('button1')
            self.last_position = None
            rospy.Service('RL_environment/start', ann4smc.srv.Start, self.start_srv)    
            rospy.Service('RL_environment/step', ann4smc.srv.Step, self.step_srv)
            rospy.spin()
        else:
            return Env.env

    def start_srv(self, request):
        self.last_position = point2array(self.limb.endpoint_pose()['position'])
        return self.last_position

    def step_srv(self, request):
        position = self.last_position or point2array(self.limb.endpoint_pose()['position'])
        target = position + np.array(request.action)
        joints = self.IK(Point(*target))
        if not joints: 
            return {'success':False, 'status_message':'Cannot perform this action'}
        else:
            self.limb.move_to_joint_positions(joints, timeout=5)
            self.last_position = self.limb.endpoint_pose()['position']
            button_pressed = self.button.is_pressed()
            reward = 1 if button_pressed else 0
            terminal = button_pressed
            return (reward, self.last_position, terminal, True, '')
        
    def IK(self, position, verbose = 0):
        ns = "ExternalTools/" + self.limb.name + "/PositionKinematicsNode/IKService"
        iksvc = rospy.ServiceProxy(ns, SolvePositionIK)
        ikreq = SolvePositionIKRequest()
        pose = PoseStamped( header=Header(stamp=rospy.Time.now(), frame_id='base'), pose=Pose(position, self.ee_orientation) )
        ikreq.pose_stamp.append(pose)
        try:
            rospy.wait_for_service(ns, 5.0)
            resp = iksvc(ikreq)
        except (rospy.ServiceException, rospy.ROSException), e:
            rospy.logerr("Service call failed: %s" % (e,))
            raise Exception

        if (resp.isValid[0]):
            limb_joints = dict(zip(resp.joints[0].name, resp.joints[0].position))
            return limb_joints


if __name__=='__main__':
    print("Initializing RL_environment... ")
    rospy.init_node('RL_environment')
    try: 
        print("Running. Ctrl-c to quit")
        Env()
    except rospy.ROSInterruptException:  
        pass 
     
