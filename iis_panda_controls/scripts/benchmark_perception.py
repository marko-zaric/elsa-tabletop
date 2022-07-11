import rospy
from rospkg import RosPack
import xacro
from geometry_msgs.msg import Pose
from gazebo_msgs.srv import SpawnModel, DeleteModel

import sys
import argparse
import numpy as np
import random
from os.path import join
import threading

# cmd arguments
parser = argparse.ArgumentParser()
parser.add_argument('--scenes', type=int, default=100, help='Number scenes to test')
FLAGS = parser.parse_args()

# globals
all_models = ["coke_can"] #['cylinder', 'box', 'sphere'] #['rectangle1', 'coke_can', 'wood_cube_7_5cm', 'wood_cube_5cm']
pose = Pose()
pose_lock = None
model_spawner = None
model_remover = None
current_models = []

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

    pose.position.x = np.random.uniform(0.1, 0.9)
    pose.position.y = np.random.uniform(-0.7, 0.7)
    pose.position.z = 0.8
    pose.orientation.w = 1.0
    pose.orientation.x = 0.0
    pose.orientation.y = 0.0
    pose.orientation.z = 0.0


def spawn_scene():
    global current_models

    num_of_models_in_scene = np.random.randint(1, 5)

    for i in range(1): #num_of_models_in_scene):
        current_model = random.choice(all_models)
        current_models.append(current_model)
        rospy.loginfo('Preparing to spawn model: {0}'.format(current_model))
        print(join("/home/marko/.gazebo/models/objects", current_model, 'model.urdf.xacro')) 
        model_urdf_xacro = join("/home/marko/.gazebo/models/objects", current_model, 'model.urdf.xacro')
        randomize_model_pose()
        model_xml, opts = randomize_model(model_urdf_xacro)


        try:
            model_spawner(current_model, model_xml, "/", pose, 'world')
        except rospy.ServiceException as e:
            rospy.logerr('Spawn model service call failed: {0}'.format(e))
        out_msg = 'Model spawned ' + str(i+1) + ' of ' + str(num_of_models_in_scene)
        rospy.loginfo(out_msg)

    return 


def delete_scene():
    global current_models

    for model in current_models:
        rospy.loginfo('Preparing to delete model: {0}'.format(model))
        try:
            while True:
                resp_delete = model_remover(model)
                rospy.loginfo(resp_delete.status_message)
                if resp_delete.success == True:
                    break
                rospy.sleep(1.0)            
        except rospy.ServiceException as e:
            rospy.logerr('Delete model service call failed: {0}'.format(e))

        rospy.loginfo('Model deleted')


def new_scene(scene):
    spawn_scene()
    #delete_scene()


if __name__ == '__main__':

    rospy.init_node('elsa_perception_benchmark')
    pose_lock = threading.Lock()
    scene = 0  

    #gazebo
    #model_spawner = rospy.ServiceProxy('/gazebo/spawn_urdf_model', SpawnModel)
    model_spawner = rospy.ServiceProxy('/gazebo/spawn_urdf_model', SpawnModel)
    model_remover = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)
    model_spawner.wait_for_service()
    model_remover.wait_for_service()

    #rospack
    rp = RosPack()


    rospy.loginfo('Perception Benchmarking with %d scenarios' % (FLAGS.scenes))

    while not rospy.is_shutdown() and scene <= FLAGS.scenes:
        rospy.sleep(2.0)
        rospy.logwarn('---------------------')
        rospy.loginfo('Scene: {0}'.format(scene))
        new_scene(scene)
        scene+=1
        rospy.loginfo('Scene over')

    rospy.loginfo('Done.')