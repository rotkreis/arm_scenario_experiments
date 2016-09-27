''' This module contains often used functions relative to baxter, including an IK solver, get limbs state and moving end effectors to a 3D pose '''

import rospy
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion, Vector3, Vector3Stamped
from std_msgs.msg import Header, String, Empty
from sensor_msgs.msg import JointState
from baxter_core_msgs.srv import SolvePositionIK, SolvePositionIKRequest
from . import utils

_real_robot = None
def is_real_robot(force = False):
    if _real_robot is None or force:
        try:
            rospy.wait_for_service('/cameras/list', timeout=1) # test if the service is available
            rospy.loginfo("Service /cameras/list exists, you're using the real robot")
            _real_robot=True
        except rospy.ROSException: # if not, it is probably cause we're using the simulator
            rospy.loginfo("Service /cameras/list does not exist, you're using the simulated robot")
            _real_robot=False    
    return _real_robot


def IK(limb, position, orientation, seed_positions = None):
    if not isinstance(position, Point): position = Point(*position)
    if not isinstance(orientation, Quaternion): orientation = Quaternion(*orientation)
    
    ns = "ExternalTools/" + limb.name + "/PositionKinematicsNode/IKService"
    iksvc = rospy.ServiceProxy(ns, SolvePositionIK)
    ikreq = SolvePositionIKRequest()
    pose = PoseStamped(
            header=Header(stamp=rospy.Time.now(), frame_id='base'),
            pose=Pose(position,orientation))
        
    ikreq.pose_stamp.append(pose)
    if seed_positions:
        seed = JointState(Header(stamp=rospy.Time.now(), frame_id='base'), limb.joint_names(), seed_positions, [0]*len(seed_positions), [0]*len(seed_positions))
        ikreq.seed_angles.append(seed)
        
    try:
        rospy.wait_for_service(ns, 5.0)
        resp = iksvc(ikreq)
    except (rospy.ServiceException, rospy.ROSException), e:
        rospy.logerr("Service call failed: %s" % (e,))
        raise Exception

    if (resp.isValid[0]):
        limb_joints = dict(zip(resp.joints[0].name, resp.joints[0].position))
        return limb_joints
    return None
        
        
def get_ee_position(limb):
    return utils.point2array(limb.endpoint_pose()['position'])
    
def get_ee_orientation(limb):
    return utils.quat2array(limb.endpoint_pose()['orientation'])
    
def move_ee_to(limb, position, orientation=None, seed_positions = None):
    if orientation is None: orientation = limb.endpoint_pose()['orientation']
    joints = IK(limb, position, orientation, seed_positions)
    if joints: 
        limb.move_to_joint_positions(joints)
        return True
    return False

