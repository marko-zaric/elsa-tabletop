import rospy
import gazebo_msgs.msg
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState
from scipy.spatial.transform import Rotation as R
import numpy as np

def callback(data):
    r = R.from_euler('xyz', [0, 0, np.pi])
    for i, name in enumerate(data.name):
        if name == 'ground_plane' or name == 'panda':
            continue
        print(i, name)
        print(data.pose[i])
        orientation = r.as_quat()
        state_msg = ModelState()
        state_msg.model_name = name
        state_msg.pose.position.x = data.pose[i].position.x
        state_msg.pose.position.y = data.pose[i].position.y
        state_msg.pose.position.z = data.pose[i].position.z
        state_msg.pose.orientation.x = orientation[0] #data.pose[i].orientation.x
        state_msg.pose.orientation.y = orientation[1] #data.pose[i].orientation.y
        state_msg.pose.orientation.z = orientation[2] #data.pose[i].orientation.z
        state_msg.pose.orientation.w = orientation[3] #data.pose[i].orientation.w

        rospy.wait_for_service('/gazebo/set_model_state')
        try:
            set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
            resp = set_state( state_msg )
        except rospy.ServiceException as e:
            print ("Service call failed:", e)

    
        
    rospy.signal_shutdown("Goal reached")
    
        

def listener():
    rospy.init_node("model_states_read", anonymous=True)
    rospy.Subscriber("/gazebo/model_states", gazebo_msgs.msg.ModelStates, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()
