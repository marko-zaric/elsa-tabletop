#!/usr/bin/env python

"""
    Returns Cartesian Pose

"""
import rospy
import quaternion
import numpy as np
from visualization_msgs.msg import *
from franka_core_msgs.msg import EndPointState, RobotState

# -- add to pythonpath for finding rviz_markers.py 
import sys, os
sys.path.append(os.path.dirname(os.path.abspath(__file__)))
# -------------------------------------------------

publish_rate = 100

JACOBIAN = None
CARTESIAN_POSE = None
CARTESIAN_VEL = None

def _on_robot_state(msg):
    """
        Callback function for updating jacobian and EE velocity from robot state
    """
    global JACOBIAN, CARTESIAN_VEL
    JACOBIAN = np.asarray(msg.O_Jac_EE).reshape(6,7,order = 'F')
    CARTESIAN_VEL = {
                'linear': np.asarray([msg.O_dP_EE[0], msg.O_dP_EE[1], msg.O_dP_EE[2]]),
                'angular': np.asarray([msg.O_dP_EE[3], msg.O_dP_EE[4], msg.O_dP_EE[5]]) }

def _on_endpoint_state(msg):
    """
        Callback function to get current end-point state
    """
    # pose message received is a vectorised column major transformation matrix
    global CARTESIAN_POSE
    cart_pose_trans_mat = np.asarray(msg.O_T_EE).reshape(4,4,order='F')

    CARTESIAN_POSE = {
        'position': cart_pose_trans_mat[:3,3],
        'orientation': quaternion.from_rotation_matrix(cart_pose_trans_mat[:3,:3]) }

def _on_shutdown():
    """
        Clean shutdown controller thread when rosnode dies.
    """
    global cartesian_state_sub, robot_state_sub

    robot_state_sub.unregister()
    cartesian_state_sub.unregister()

if __name__ == "__main__":
    # global goal_pos, goal_ori, ctrl_thread

    rospy.init_node("read_panda_cartesian")
    
    cartesian_state_sub = rospy.Subscriber(
        'panda_simulator/custom_franka_state_controller/tip_state',
        EndPointState,
        _on_endpoint_state,
        queue_size=1,
        tcp_nodelay=True)

    robot_state_sub = rospy.Subscriber(
        'panda_simulator/custom_franka_state_controller/robot_state',
        RobotState,
        _on_robot_state,
        queue_size=1,
        tcp_nodelay=True)


    # wait for messages to be populated before proceeding
    rospy.loginfo("Subscribing to robot state topics...")
    while (True):
        if not (JACOBIAN is None or CARTESIAN_POSE is None):
            break
    rospy.loginfo("Recieved messages; Starting Demo.")
    
    rospy.on_shutdown(_on_shutdown)
    rate = rospy.Rate(publish_rate)

    while not rospy.is_shutdown():
        rospy.loginfo("Position:")
        rospy.loginfo(CARTESIAN_POSE["position"])
        rospy.loginfo("Orientation:")
        rospy.loginfo(CARTESIAN_POSE["orientation"])
        rospy.spin()    