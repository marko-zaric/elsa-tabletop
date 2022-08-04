import rospy
import sys
from tomlkit import string
sys.path.append("/home/marko/Desktop/IIS_Research/catkin_workspaces/panda_catkin_ws/src/panda_simulator/iis_panda_controls/scripts")
import argparse
import numpy as np
import random
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState
from scipy.spatial.transform import Rotation as R
import numpy as np
import pandas as pd 


parser = argparse.ArgumentParser()
parser.add_argument('--in_file', type=string, help='Benchmark Out file')
parser.add_argument('--scene', type=int, help='Scene to look at')
FLAGS = parser.parse_args()


df = pd.read_csv(FLAGS.in_file)

TO_SPAWN = []

for i in range(len(df)):
    if df.iloc[i,0] == FLAGS.scene:
        TO_SPAWN.append(list(df.iloc[i,:]))


publish_rate = 100



OBJECT_LIST = ["can1", "can2", "brick1", "brick2", "cube_small", "cube_large"]


OUT_OF_FRAME_POSE = [[2.0,2.0], [2.0, -2.0], [-2.0, -2.0], [-2.0, 2.0], [2.5, 2.5], [2.0, 2.5]]


def move_out_of_frame():
    out_of_frame_poses = 0
    for name in OBJECT_LIST:
        state_msg = ModelState()
        state_msg.pose.position.x = OUT_OF_FRAME_POSE[out_of_frame_poses][0]
        state_msg.pose.position.y = OUT_OF_FRAME_POSE[out_of_frame_poses][1]
        out_of_frame_poses += 1
        state_msg.model_name = name
        state_msg.pose.position.z = 0.01
        state_msg.pose.orientation.x = 0
        state_msg.pose.orientation.y = 0
        state_msg.pose.orientation.z = 1
        state_msg.pose.orientation.w = 0

        rospy.wait_for_service('/gazebo/set_model_state')
        try:
            set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
            resp = set_state( state_msg )
        except rospy.ServiceException as e:
            print ("Service call failed:", e)

def new_scene():
    move_out_of_frame()
    
    for object in TO_SPAWN:
        state_msg = ModelState()
        r = R.from_euler('xyz', [0, 0, object[5]])
        orientation = r.as_quat()
        state_msg.model_name = object[1]
        #print(name, euler_z_angle[i] % np.pi)
        state_msg.pose.position.x = object[2]
        state_msg.pose.position.y = object[3]
        state_msg.pose.position.z = 0.01
        state_msg.pose.orientation.x = orientation[0] 
        state_msg.pose.orientation.y = orientation[1]
        state_msg.pose.orientation.z = orientation[2] 
        state_msg.pose.orientation.w = orientation[3] 
        rospy.wait_for_service('/gazebo/set_model_state')
        try:
            set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
            resp = set_state( state_msg )
        except rospy.ServiceException as e:
            print ("Service call failed:", e)

if __name__ == '__main__':

    rospy.init_node('generate_scene')
    rate = rospy.Rate(10)

    new_poses = new_scene()

    rospy.loginfo('Done.')