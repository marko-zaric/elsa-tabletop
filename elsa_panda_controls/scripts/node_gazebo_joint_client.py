#!/usr/bin/env python
import rospy
import numpy as np
# from visualization_msgs.msg import *
from elsa_panda_controls.srv import JointConfig
from elsa_panda_controls.msg import StrArray
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

def publish():
    rospy.init_node("control_panda", anonymous=True)
    rate = rospy.Rate(publish_rate)

    joints = StrArray()
    joints.list_of_strings = ["panda_joint1", "panda_joint2", "panda_joint3", "panda_joint4" ,"panda_joint5", "panda_joint6", "panda_joint7"]
    values = [-0.501997270431858, 0.765960214005434, 0.12069809659440667, -2.4372512139611415, 2.732772001022027, 1.5589091425047412, -2.4802445685953426]

    try:
        send_specific_joint_config = rospy.ServiceProxy("send_joint_config", JointConfig)
        response = send_specific_joint_config(joints, values)
        print(response)
    except rospy.ServiceException as e:
        print("Service failed %s", e)




if __name__ == "__main__":
    # global ctrl_thread
    try:
        publish()
    except rospy.ROSInterruptException:
        pass 
 
  