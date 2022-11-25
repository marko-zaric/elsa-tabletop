#!/usr/bin/env python
import rospy
import numpy as np
# from visualization_msgs.msg import *
from franka_core_msgs.msg import JointCommand
import argparse
from sensor_msgs.msg import JointState
from elsa_panda_controls.msg import StrArray
from elsa_panda_controls.srv import JointConfig, JointConfigResponse

joint_limits = {
    "panda_joint1": [-2.8973, 2.8973],
    "panda_joint2": [-1.7628, 1.7628],
    "panda_joint3": [-2.8973, 2.8973],
    "panda_joint4": [-3.0718, -0.0698],
    "panda_joint5": [-2.8973, 2.8973],
    "panda_joint6": [-0.0175, 3.7525],
    "panda_joint7": [-2.8973, 2.8973],
}


publish_rate = 100

CURRENT_POSITION = None
JOINTS = None
JOINT_VALUES = None

def _on_robot_state(msg):
    """
        Callback function for updating joint_states
    """
    global CURRENT_POSITION, JOINTS, JOINT_VALUES
    if JOINTS != None:
        cur_positions = []
        for joint in JOINTS:
            index_joint = msg.name.index(joint)
            cur_positions.append(msg.position[index_joint])
        CURRENT_POSITION = cur_positions


def callback(request):
    rate = rospy.Rate(publish_rate)

    global JOINTS, JOINT_VALUES
    JOINTS_REQEUEST = StrArray()
    JOINTS_REQEUEST = request.joints
    JOINT_VALUES = request.values
    JOINTS = JOINTS_REQEUEST.list_of_strings
    # Also create a publisher to publish joint commands
    joint_command_publisher = rospy.Publisher(
            'panda_simulator/motion_controller/arm/joint_commands',
            JointCommand,
            tcp_nodelay=True,
            queue_size=1)

    robot_state_sub = rospy.Subscriber(
        'joint_states',
        JointState,
        _on_robot_state,
        queue_size=1,
        tcp_nodelay=True)

    # wait for messages to be populated before proceeding
    rospy.loginfo("Subscribing to robot state topics...")
    while (True):
        if not (CURRENT_POSITION is None):
            break
    rospy.loginfo("Recieved messages; Starting Move.")

    command_msg = JointCommand()
    command_msg.mode = JointCommand.POSITION_MODE
    command_msg.names = JOINTS
    command_msg.position = JOINT_VALUES
    while np.linalg.norm(np.array(CURRENT_POSITION) - np.array(JOINT_VALUES)) > 10**-3:
        joint_command_publisher.publish(command_msg)
        rate.sleep()

    return JointConfigResponse(True)

def send_joint_config():
    rospy.init_node("joint_config_server")
    service = rospy.Service('send_joint_config', JointConfig, callback)
    rospy.loginfo("Joint Config service started...")
    
    rospy.spin()


if __name__ == "__main__":
    # global ctrl_thread
    try:
        send_joint_config()
    except rospy.ROSInterruptException:
        pass 
 
  