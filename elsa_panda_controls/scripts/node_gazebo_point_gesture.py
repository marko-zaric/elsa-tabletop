#!/usr/bin/env python
import copy
import rospy
import numpy as np
import threading
from franka_core_msgs.msg import EndPointState, JointCommand, RobotState
from sensor_msgs.msg import JointState
import quaternion
from elsa_panda_controls.msg import StrArray
from elsa_panda_controls.srv import JointConfig
import argparse

parser = argparse.ArgumentParser()
parser.add_argument("-p", "--position", required=False, help="Position of the center of object where to point at.", type=float, nargs="+", default=[0.8, 0.5, 0.06])
parser.add_argument("-g", "--goal_is_object", required=False, help="Decides on Joints to point to object or space", type=int, default=1)
parser.add_argument("-o", "--orientation", required=False, help="orients robot arm just enter quaternion as float numbers", type=float, nargs="+", default=[0.0, -1.0, 0.0, 0.0])
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
CURRENT_POSITION = None
GOAL_IS_OBJECT = args.goal_is_object

if GOAL_IS_OBJECT:
    JOINTS = ['panda_joint2','panda_joint3','panda_joint4','panda_joint5']
else:
    JOINTS = ['panda_joint1','panda_joint2','panda_joint3','panda_joint4','panda_joint5']
    


TARGET = None

def _on_joint_state(msg):
    """
        Callback function for updating joint_states
    """
    global CURRENT_POSITION, JOINTS, GOAL_IS_OBJECT
    if JOINTS != None:
        cur_positions = []
        for joint in JOINTS:
            index_joint = msg.name.index(joint)
            cur_positions.append(msg.position[index_joint])
        CURRENT_POSITION = cur_positions

def _on_robot_state(msg):
    """
        Callback function for updating jacobian and EE velocity from robot state
    """
    global JACOBIAN, CARTESIAN_VEL, CURRENT_POSITION, JOINTS
    total_jacobian = np.asarray(msg.O_Jac_EE).reshape(6,7,order = 'F')
    if GOAL_IS_OBJECT:
        JACOBIAN = np.delete(total_jacobian, obj=(0, 5, 6), axis=1)
    else:
        JACOBIAN = np.delete(total_jacobian, obj=(5, 6), axis=1)

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
    quat_des = np.quaternion(quat_des[0], quat_des[1],quat_des[2],quat_des[3])
    curr_mat = quaternion.as_rotation_matrix(quat_curr)
    des_mat = quaternion.as_rotation_matrix(quat_des)
    rel_mat = des_mat.T.dot(curr_mat)
    rel_quat = quaternion.from_rotation_matrix(rel_mat)
    vec = quaternion.as_float_array(rel_quat)[1:]
    if rel_quat.w < 0.0:
        vec = -vec
        
    return -des_mat.dot(vec)

joint_limits = {
    "panda_joint1": [-2.8973, 2.8973],
    "panda_joint2": [-1.7628, 1.7628],
    "panda_joint3": [-2.8973, 2.8973],
    "panda_joint4": [3.212, 6.2], # very strange limits
    "panda_joint5": [-2.8973, 2.8973],
    "panda_joint6": [0.0714, 3.7525], # again strange limit
    "panda_joint7": [-2.8973, 2.8973],
}


publish_rate = 100

def calculate_goal_point(pos):
    object_center = pos
    joint1_angle = np.arctan2(object_center[0], object_center[1])
    distance_robot_object_2D = np.sqrt(object_center[0]**2 + object_center[1]**2)
    y_target = 0.8 * distance_robot_object_2D * np.sin(joint1_angle)
    x_target = 0.8 * distance_robot_object_2D * np.cos(joint1_angle)
    print("TARGET: ", [y_target, x_target, 0.2 ])
    return (joint1_angle, [y_target , x_target , 0.2])

def control_thread(rate, command_msg, joint_command_publisher):
    """
        Actual control loop. Uses goal pose from the feedback thread
        and current robot states from the subscribed messages to compute
        task-space force, and then the corresponding joint torques.
    """
    while not rospy.is_shutdown():
        error = 100.
        goal_pos = np.array(TARGET)
        # if args.orientation[0] == 0 and args.orientation[1] == 0 and args.orientation[2] == 0 and args.orientation[3] == 0:
        #     goal_ori = CARTESIAN_POSE['orientation'] #np.quaternion(q.w, q.x,q.y,q.z)
        # else:
        goal_ori = np.array(args.orientation)
        
        # Position Control
        damping = 0.001
        alpha = 0.05
        while error > 0.03:
            curr_pose = copy.deepcopy(CARTESIAN_POSE)
            curr_pos, curr_ori = curr_pose['position'],curr_pose['orientation']
            delta_pos = (goal_pos - curr_pos).reshape([3,1])

            J = copy.deepcopy(JACOBIAN)
            
            J_pinv = J.T@ np.linalg.inv(J@J.T + damping**2*np.identity(6))  #np.linalg.pinv(J)
            J_pinv_pos = J_pinv[:, :3]

            e = np.atleast_2d(goal_pos - curr_pos).T

            update = +alpha*J_pinv_pos@e
            new_position = np.array(CURRENT_POSITION) + update.flatten()
            command_msg.position = list(new_position)
            print("New Joint Positions: ", command_msg.position)
            
            joint_command_publisher.publish(command_msg)
            error = np.linalg.norm(delta_pos)
            print("Error: ", error)
            rate.sleep()
 
        rospy.signal_shutdown("Goal reached")


def perform_gesture():
    global TARGET, JOINTS, GOAL_IS_OBJECT
    # 
    joint1_angle, TARGET  = calculate_goal_point(args.position)
    rospy.init_node("point_gesture_node", anonymous=True)
    # global ctrl_thread
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
    

    joint_state_sub = rospy.Subscriber(
        'joint_states',
        JointState,
        _on_joint_state,
        queue_size=1,
        tcp_nodelay=True)

    rate = rospy.Rate(publish_rate)
    

    # wait for messages to be populated before proceeding
    rospy.loginfo("Waiting for joint configuration service...")

    rospy.wait_for_service("send_joint_config")
    rospy.loginfo("Joint configuration service found ...")

    joints = StrArray()
    joints.list_of_strings = ["panda_joint1", "panda_joint6"]
    print(GOAL_IS_OBJECT)
    if GOAL_IS_OBJECT:
        values = [joint1_angle, 3.14]
    else:
        values = [0.0, 3.14]

    print("First step")
    try:
        send_specific_joint_config = rospy.ServiceProxy("send_joint_config", JointConfig)
        response = send_specific_joint_config(joints, values)
        print(response)
    except rospy.ServiceException as e:
        print("Service failed %s", e)

    print("second step")
    # create joint command message and fix its type to joint torque mode
    command_msg = JointCommand()
    command_msg.names = JOINTS
    command_msg.mode = JointCommand.POSITION_MODE

     # Also create a publisher to publish joint commands
    joint_command_publisher = rospy.Publisher(
            'panda_simulator/motion_controller/arm/joint_commands',
            JointCommand,
            tcp_nodelay=True,
            queue_size=1)

    # wait for messages to be populated before proceeding
    rospy.loginfo("Subscribing to robot state topics...")
    while (True):
        if not (JACOBIAN is None or CARTESIAN_POSE is None or JOINTS is None):
            break
    rospy.loginfo("Recieved messages; Starting Demo.")

    rate = rospy.Rate(publish_rate)

    ctrl_thread = threading.Thread(target=control_thread, args = [rate, command_msg, joint_command_publisher])
    ctrl_thread.start()


    rospy.spin()  


if __name__ == "__main__":
    try:
        perform_gesture()
    except rospy.ROSInterruptException:
        pass 
 
  