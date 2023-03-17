import rospy
from rospkg import RosPack
import xacro
from geometry_msgs.msg import Pose, Point, WrenchStamped
from std_srvs.srv import SetBool
from gazebo_msgs.srv import SpawnModel, DeleteModel, SetModelState, SetModelStateRequest, BodyRequest, BodyRequestRequest
from gazebo_msgs.msg import ModelStates
from visualization_msgs.msg import Marker
from std_msgs.msg import String, Bool, Int32

import argparse
import numpy as np
from copy import deepcopy
import random
from os.path import join
import threading


# cmd arguments
parser = argparse.ArgumentParser()
parser.add_argument('--episodes', type=int, default=1, help='Number of episodes to run')
parser.add_argument('--model', type=str, default='box', help='Set object') # all_models = ['cylinder', 'box', 'sphere']
FLAGS = parser.parse_args()

# globals

pose = Pose()
pose_lock = None
model_spawner = None
model_remover = None
current_model = None
rp = None


def waiter_callback(msg):
    global wait

    wait = msg.data


def randomize_model(model_urdf_xacro):
    # Process the file once using xacro to retrieve arguments
    rospy.loginfo('Randomizing model')

    opts, _ = xacro.process_args([model_urdf_xacro])
    model_xacro = xacro.process_file(model_urdf_xacro, **vars(opts))

    # Check xacro args for corresponding userdata input keys and if
    # matches are found, fill out the args with the input key values.
    model_xacro_args = vars(opts)['mappings']
    for arg, arg_val in model_xacro_args.items():
        if arg == 'rpy':
            model_xacro_args[arg] = "0.0 0.0 0.0"
        elif arg == 'xyz':
            model_xacro_args[arg] = "0.0 0.0 0.0"
        else:
            multiplicator = np.random.uniform(3., 5.)
            model_xacro_args[arg] = str(float(arg_val)*multiplicator)

    # Process the file once more using the new args
    new_opts = vars(opts)
    new_opts['mappings'] = model_xacro_args
    model_xacro = xacro.process_file(model_urdf_xacro, **new_opts)
    # Dump to xml and return
    return model_xacro.toxml(), new_opts


def randomize_model_pose():
    global pose

    translator_x = np.random.uniform(-0.05, 0.05)
    translator_y = np.random.uniform(-0.05, 0.05)
    pose.position.x = np.random.uniform(0.7, 0.9) + translator_x
    pose.position.y = np.random.uniform(0.7, 0.9) + translator_y
    pose.position.z = 0.8
    pose.orientation.w = 1.0
    pose.orientation.x = 0.0
    pose.orientation.y = 0.0
    pose.orientation.z = 0.0


def spawn_model():
    global current_model

    current_model = FLAGS.model
    rospy.loginfo('Preparing to spawn model: {0}'.format(current_model))
    model_urdf_xacro = join(rp.get_path('elsa_training_data_generation'), 'models/objects', current_model, 'model.urdf.xacro')
    randomize_model_pose()
    model_xml, opts = randomize_model(model_urdf_xacro)

    try:
        model_spawner(current_model, model_xml, "/", pose, 'world')
    except rospy.ServiceException as e:
        rospy.logerr('Spawn model service call failed: {0}'.format(e))

    rospy.loginfo('Model spawned')
    rospy.sleep(1)


    return opts


def delete_model():
    rospy.loginfo('Preparing to delete model: {0}'.format(current_model))
    try:
        while True:
            resp_delete = model_remover(current_model)
            rospy.loginfo(resp_delete.status_message)
            if resp_delete.success == True:
                break
            rospy.sleep(1.0)            
    except rospy.ServiceException as e:
        rospy.logerr('Delete model service call failed: {0}'.format(e))

    rospy.loginfo('Model deleted')

def model_pose_callback(msg):
    global pose

    if current_model in msg.name:
        idx = msg.name.index(current_model)
        with pose_lock:
            pose = msg.pose[idx]


def generate_episode(episode):
    opts = spawn_model()
    rospy.loginfo('Deliberate pause...')
    rospy.sleep(10)
    delete_model()


if __name__ == '__main__':

    rospy.init_node('elsa_training_data_generator')
    pose_lock = threading.Lock()
    episode = 1   

    #gazebo
    model_spawner = rospy.ServiceProxy('/gazebo/spawn_urdf_model', SpawnModel)
    model_remover = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)
    model_spawner.wait_for_service()
    model_remover.wait_for_service()
    model_pose_tracker = rospy.Subscriber('/gazebo/model_states_throttled', ModelStates, queue_size=1, callback=model_pose_callback, tcp_nodelay=True)
      #rospack
    rp = RosPack()

    rospy.loginfo('Doing %d episodes' % (FLAGS.episodes))

    while not rospy.is_shutdown() and episode <= FLAGS.episodes:
        rospy.sleep(5.0)
        rospy.logwarn('---------------------')
        rospy.loginfo('Episode: {0}'.format(episode))
        generate_episode(episode)
        episode+=1
        rospy.loginfo('Episode over')

    rospy.loginfo('Done.')
