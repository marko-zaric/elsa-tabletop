#!/usr/bin/env python
import rospy
import numpy as np
# from visualization_msgs.msg import *
from franka_core_msgs.msg import JointCommand
import argparse
from sensor_msgs.msg import JointState

parser = argparse.ArgumentParser()
parser.add_argument("-j", "--joint", required=False, help="Joint to publish the config on.", type=str, default="panda_joint1")
parser.add_argument("-v", "--value", required=False, help="Float value for joint", type=float, default=0)
args = parser.parse_args()

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

def _on_robot_state(msg):
    """
        Callback function for updating joint_states
    """
    global CURRENT_POSITION
    index_joint = msg.name.index(args.joint)
    CURRENT_POSITION = msg.position[index_joint]


def test_joint():
    rospy.init_node("single_joint_control", anonymous=True)
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

    command_msg = JointCommand()
    command_msg.mode = JointCommand.POSITION_MODE
    command_msg.names = [args.joint]
    command_msg.position = [args.value]
    while np.linalg.norm(np.array(CURRENT_POSITION) - np.array(args.value)) > 10**-3:
        joint_command_publisher.publish(command_msg)
        rate.sleep()

if __name__ == "__main__":
    # global ctrl_thread
    try:
        test_joint()
    except rospy.ROSInterruptException:
        pass 
 
  