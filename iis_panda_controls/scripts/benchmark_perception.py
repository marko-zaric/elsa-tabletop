import rospy
import sys
from tomlkit import string
# sys.path.append("/home/marko/Desktop/IIS_Research/catkin_workspaces/panda_catkin_ws/src/panda_simulator/iis_panda_controls/scripts")
import xacro
from geometry_msgs.msg import Pose
from gazebo_msgs.srv import SpawnModel
import argparse
import numpy as np
import random
from os.path import join
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState
from scipy.spatial.transform import Rotation as R
import numpy as np
from itertools import product
import random
import copy
from iis_panda_controls.msg import BB_Scene
import csv
from datetime import datetime



# cmd arguments
parser = argparse.ArgumentParser()
parser.add_argument('--scenes', type=int, default=1, help='Number scenes to test')
parser.add_argument('--out_folder', type=string, default='/home/marko/Desktop/IIS_Research/benchmarking/', help='Folder where the simulation output will be stored')
FLAGS = parser.parse_args()

# benchmark results
CSV_FILE = open(FLAGS.out_folder + str(datetime.now())+ '.csv', 'w')
CSV_WRITER = csv.writer(CSV_FILE)

# globals
# all_models = #['cylinder', 'box', 'sphere']
pose = Pose()
pose_lock = None
model_spawner = None

# POSE_INFO = None

publish_rate = 100

neutral_position = [-0.34229236109164773, -1.3855501963161174, -0.6734278254553914, -2.3943482548210486, -0.728386864031318, 1.1721292985506233, -0.035632236512602944]

CURRENT_POSITION = None
BOUNDING_BOXES = None
COMPUTATION_TIME = None
OBSERVED = 0 # 0 = make new scene, 1= waiting to be observed, 2 process observation
OBJECT_LIST = ["can1", "can2", "brick1", "brick2", "cube_small", "cube_large"]


OBJ_DIMENSIONS = {'can1':[0.06701,0.06701, 0.1239],
                'can2':[0.06701,0.06701, 0.1239],
                'brick1':[0.06, 0.12, 0.07],
                'brick2':[0.06, 0.12, 0.07],
                'cube_small':[0.05, 0.05, 0.05],
                'cube_large':[0.075, 0.075, 0.075]}


OUT_OF_FRAME_POSE = [[2.0,2.0], [2.0, -2.0], [-2.0, -2.0], [-2.0, 2.0], [2.5, 2.5], [2.0, 2.5]]

y_grid = [-0.15, 0, 0.15, 0.3]
x_grid = [0.2, 0.35, 0.5, 0.65]
XY_GRID = list(product(x_grid, y_grid))



def read_camera(data):
    global BOUNDING_BOXES, COMPUTATION_TIME, OBSERVE
    BOUNDING_BOXES = data.boundingboxes


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
        #current_model = random.choice(all_models)
        current_model = 'sphere'
        rospy.loginfo('Preparing to spawn model: {0}'.format(current_model))
        model_urdf_xacro = join("/home/marko/.gazebo/models/objects", current_model, 'model.urdf.xacro')
        randomize_model_pose()
        model_xml, opts = randomize_model(model_urdf_xacro)



        try:
            model_spawner(current_model, model_xml, "/", pose, 'world')
            #model_spawner(obj_name, model_xacro.toxml(), "/", pose, 'world')
        except rospy.ServiceException as e:
            rospy.logerr('Spawn model service call failed: {0}'.format(e))
        out_msg = 'Model spawned ' + str(i+1) + ' of ' + str(num_of_models_in_scene)
        rospy.loginfo(out_msg)

    return 

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
    new_poses = dict()
    xy_grid = copy.copy(XY_GRID)
    in_frame = np.random.randint(low=0, high=2, size=7)
    euler_z_angle = np.random.random(size=7) * np.pi - (np.pi / 2)
    #print("winkel: ", euler_z_angle)
    move_out_of_frame()
    for i, name in enumerate(OBJECT_LIST):
        state_msg = ModelState()
        if in_frame[i] != 0:
            xy = random.choice(xy_grid)
            xy_grid.pop(xy_grid.index(xy))
            r = R.from_euler('xyz', [0, 0, euler_z_angle[i]])
            orientation = r.as_quat()
            state_msg.model_name = name
            #print(name, euler_z_angle[i] % np.pi)
            state_msg.pose.position.x = xy[0]
            state_msg.pose.position.y = xy[1]
            state_msg.pose.position.z = 1.03
            state_msg.pose.orientation.x = orientation[0] #data.pose[i].orientation.x
            state_msg.pose.orientation.y = orientation[1] #data.pose[i].orientation.y
            state_msg.pose.orientation.z = orientation[2] #data.pose[i].orientation.z
            state_msg.pose.orientation.w = orientation[3] #data.pose[i].orientation.w

            rospy.wait_for_service('/gazebo/set_model_state')
            try:
                set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
                resp = set_state( state_msg )
                new_poses[name] = [state_msg.pose,  euler_z_angle[i]]
            except rospy.ServiceException as e:
                print ("Service call failed:", e)
    return new_poses

        

# def model_pose_tracker(data):
#     global POSE_INFO
#     POSE_INFO = data

        
def evaluate(new_poses, bb_camera, comp_time, scene):
    global CSV_WRITER
    # print(bb_camera)
    # print(len(bb_camera)-1)
    for pose in new_poses:
        if np.abs(new_poses[pose][0].position.x) > 1:
            # out of view 
            continue
        print(pose)
        # print(new_poses[pose][0].position.x, "/", new_poses[pose][0].position.y)
        # find closest object 
        min_index = 0
        min_diff = 1000
        for i, cam_pose in enumerate(bb_camera):
            diff = np.abs(cam_pose.x - new_poses[pose][0].position.x) + np.abs(cam_pose.y - new_poses[pose][0].position.y)
            if diff < min_diff:
                min_diff = diff
                min_index = i
        round = False
        if pose.startswith("can"):
            angle = 0.0
            round = True
        else:
            angle = new_poses[pose][1]

        round_percept = False
        if bb_camera[min_index].phi == 0.0:
           round_percept = True 

        CSV_WRITER.writerow([scene, pose, new_poses[pose][0].position.x, new_poses[pose][0].position.y, new_poses[pose][0].position.z,
                            angle, OBJ_DIMENSIONS[pose][0], OBJ_DIMENSIONS[pose][1], 
                            OBJ_DIMENSIONS[pose][2], bb_camera[min_index].x, bb_camera[min_index].y, bb_camera[min_index].z,
                            bb_camera[min_index].phi, bb_camera[min_index].dx, bb_camera[min_index].dy, bb_camera[min_index].dz, round, round_percept])   

        
        
        


if __name__ == '__main__':

    rospy.init_node('elsa_perception_benchmark')
    rate = rospy.Rate(10)

    # rospy.Subscriber("/gazebo/model_states", gazebo_msgs.msg.ModelStates, queue_size=1, callback=model_pose_tracker)
    rospy.Subscriber("/scene_description",BB_Scene , queue_size=1, callback=read_camera)
    scene = 0 


    #model spawner
    model_spawner = rospy.ServiceProxy('/gazebo/spawn_urdf_model', SpawnModel)
    model_spawner.wait_for_service()

    rospy.loginfo('Perception Benchmarking with %d scenarios' % (FLAGS.scenes))
    #spawn_scene()
    count_msgs = 0
    # evaluation loop
    while not rospy.is_shutdown() and scene < FLAGS.scenes:  
        if count_msgs == 0:
            rospy.logwarn('---------------------')
            rospy.loginfo('Scene: {0}'.format(scene+1))
            new_poses = new_scene()
            rospy.loginfo('New scene created')
            count_msgs += 1
        elif count_msgs < 4:
            rospy.wait_for_message("/scene_description", BB_Scene)
            count_msgs += 1
        else:
            snapshot_bb = copy.copy(BOUNDING_BOXES)
            snapshot_computation_time = COMPUTATION_TIME
            rospy.loginfo('Comp time snapshoted')
            evaluate(new_poses, snapshot_bb, snapshot_computation_time, scene)
            rospy.loginfo('Evaluation done')
            scene+=1
            rospy.loginfo('Scene over')
            count_msgs = 0

    CSV_FILE.close()
    rospy.loginfo('Done.')