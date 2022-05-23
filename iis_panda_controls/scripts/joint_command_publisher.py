#!/usr/bin/env python
import copy
import rospy
import threading
import quaternion
import numpy as np
from visualization_msgs.msg import *
from franka_core_msgs.msg import EndPointState, JointCommand, RobotState
import argparse

parser = argparse.ArgumentParser()
parser.add_argument("position", help="Moves robot arm to set position just enter x y z as float numbers", nargs="+", type=float)
args = parser.parse_args()
# -------------------------------------------------
# --------- Modify as required ------------
# Task-space controller parameters
# stiffness gains
P_pos = 50.
P_ori = 25.
# damping gains
D_pos = 10.
D_ori = 1.
# -----------------------------------------
# ------- Default pose values -------------
default_orientation = np.quaternion(0.00562735094448318, -0.999728357180464, 0.00324361880728706, 0.0223835588687537)
# -----------------------------------------
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

def quatdiff_in_euler(quat_curr, quat_des):
    """
        Compute difference between quaternions and return 
        Euler angles as difference
    """
    curr_mat = quaternion.as_rotation_matrix(quat_curr)
    des_mat = quaternion.as_rotation_matrix(quat_des)
    rel_mat = des_mat.T.dot(curr_mat)
    rel_quat = quaternion.from_rotation_matrix(rel_mat)
    vec = quaternion.as_float_array(rel_quat)[1:]
    if rel_quat.w < 0.0:
        vec = -vec
        
    return -des_mat.dot(vec)

def control_thread(rate):
    """
        Actual control loop. Uses goal pose from the feedback thread
        and current robot states from the subscribed messages to compute
        task-space force, and then the corresponding joint torques.
    """
    while not rospy.is_shutdown():
        error = 100.
        
        goal_pos = np.array(args.position)
        goal_ori = CARTESIAN_POSE['orientation'] #np.quaternion(q.w, q.x,q.y,q.z)
        while error > 0.005:
            curr_pose = copy.deepcopy(CARTESIAN_POSE)
            curr_pos, curr_ori = curr_pose['position'],curr_pose['orientation']

            curr_vel = (CARTESIAN_VEL['linear']).reshape([3,1])
            curr_omg = CARTESIAN_VEL['angular'].reshape([3,1])

            delta_pos = (goal_pos - curr_pos).reshape([3,1])
            delta_ori = quatdiff_in_euler(curr_ori, goal_ori).reshape([3,1])

            # Desired task-space force using PD law
            F = np.vstack([P_pos*(delta_pos), P_ori*(delta_ori)]) - \
                np.vstack([D_pos*(curr_vel), D_ori*(curr_omg)])

            error = np.linalg.norm(delta_pos) + np.linalg.norm(delta_ori)
            
            J = copy.deepcopy(JACOBIAN)

            # joint torques to be commanded
            tau = np.dot(J.T,F)
            # publish joint commands
            command_msg.effort = tau.flatten()
            joint_command_publisher.publish(command_msg)
            rate.sleep()
        rospy.signal_shutdown("Goal reached")

# def _on_shutdown():
#     """
#         Clean shutdown controller thread when rosnode dies.
#     """
#     global ctrl_thread, cartesian_state_sub, \
#         robot_state_sub, joint_command_publisher
#     if ctrl_thread.is_alive():
#         ctrl_thread.join()

#     robot_state_sub.unregister()
#     cartesian_state_sub.unregister()
#     joint_command_publisher.unregister()

if __name__ == "__main__":
    # global ctrl_thread

    rospy.init_node("control_panda")
    
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

    # create joint command message and fix its type to joint torque mode
    command_msg = JointCommand()
    command_msg.names = ['panda_joint1','panda_joint2','panda_joint3',\
        'panda_joint4','panda_joint5','panda_joint6','panda_joint7']
    command_msg.mode = JointCommand.TORQUE_MODE
    
    # Also create a publisher to publish joint commands
    joint_command_publisher = rospy.Publisher(
            'panda_simulator/motion_controller/arm/joint_commands',
            JointCommand,
            tcp_nodelay=True,
            queue_size=1)

    # wait for messages to be populated before proceeding
    rospy.loginfo("Subscribing to robot state topics...")
    while (True):
        if not (JACOBIAN is None or CARTESIAN_POSE is None):
            break
    rospy.loginfo("Recieved messages; Starting Demo.")

    # rospy.on_shutdown(_on_shutdown)
    rate = rospy.Rate(publish_rate)
    ctrl_thread = threading.Thread(target=control_thread, args = [rate])
    ctrl_thread.start()

    rospy.spin()    