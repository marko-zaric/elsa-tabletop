#!/usr/bin/env python
import rospy
import numpy as np
# from visualization_msgs.msg import *
from franka_core_msgs.msg import JointCommand
import argparse
from sensor_msgs.msg import JointState

# parser = argparse.ArgumentParser()
# parser.add_argument("-j", "--joint", required=False, help="Tests range of certain joint.", type=str, default="panda_joint1")
# args = parser.parse_args()

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

CURRENT_POSITION = None

def _on_robot_state(msg):
    """
        Callback function for updating joint_states
    """
    global CURRENT_POSITION
    index_joint = msg.name.index("panda_joint6")
    CURRENT_POSITION = msg.position[index_joint]


def perform_gesture():
    rospy.init_node("control_panda", anonymous=True)
    rate = rospy.Rate(publish_rate)
    
    
    
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

    for position in [1.57, 1.3, 1.57, 1.3, 1.57]:
        command_msg = JointCommand()
        command_msg.mode = JointCommand.POSITION_MODE
        command_msg.names = ["panda_joint6"]
        command_msg.position = [position]

        while np.linalg.norm(np.array(CURRENT_POSITION) - np.array(position)) > 10**-3:
            joint_command_publisher.publish(command_msg)
            rate.sleep()

if __name__ == "__main__":
    # global ctrl_thread
    try:
        perform_gesture()
    except rospy.ROSInterruptException:
        pass 
 
  