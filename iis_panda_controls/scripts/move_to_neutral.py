#!/usr/bin/env python
import numpy as np
import rospy
from visualization_msgs.msg import *
from franka_core_msgs.msg import JointCommand
from sensor_msgs.msg import JointState

publish_rate = 100

neutral_position = [-0.34229236109164773, -1.3855501963161174, -0.6734278254553914, -2.3943482548210486, -0.728386864031318, 1.1721292985506233, -0.035632236512602944]

CURRENT_POSITION = None

def _on_robot_state(msg):
    """
        Callback function for updating joint_states
    """
    global CURRENT_POSITION
    CURRENT_POSITION = msg.position[2:]


def move_to_neutral():
    rospy.init_node("control_panda", anonymous=True)
    rate = rospy.Rate(publish_rate)
    
    # create joint command message and fix its type to joint torque mode
    command_msg = JointCommand()
    command_msg.names = ['panda_joint1','panda_joint2','panda_joint3',\
        'panda_joint4','panda_joint5','panda_joint6','panda_joint7']
    command_msg.mode = JointCommand.POSITION_MODE
    command_msg.position = neutral_position
    
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

    while np.linalg.norm(np.array(CURRENT_POSITION) - np.array(neutral_position)) > 10**-3:
        joint_command_publisher.publish(command_msg)
        rate.sleep()

if __name__ == "__main__":
    # global ctrl_thread
    try:
        move_to_neutral()
    except rospy.ROSInterruptException:
        pass 
 
  