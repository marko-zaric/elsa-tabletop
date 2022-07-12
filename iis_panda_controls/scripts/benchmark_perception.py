import rospy
import xacro
import gazebo_msgs
from geometry_msgs.msg import Pose
from gazebo_msgs.srv import SpawnModel
from franka_core_msgs.msg import JointCommand
from sensor_msgs.msg import JointState
import argparse
import numpy as np
import random
from os.path import join
import threading
import os
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState
from scipy.spatial.transform import Rotation as R
import numpy as np
from itertools import product
import random



# cmd arguments
parser = argparse.ArgumentParser()
parser.add_argument('--scenes', type=int, default=1, help='Number scenes to test')
FLAGS = parser.parse_args()

# globals
# all_models = #['cylinder', 'box', 'sphere']
pose = Pose()
pose_lock = None
model_spawner = None

POSE_INFO = None

publish_rate = 100

neutral_position = [-0.34229236109164773, -1.3855501963161174, -0.6734278254553914, -2.3943482548210486, -0.728386864031318, 1.1721292985506233, -0.035632236512602944]

CURRENT_POSITION = None

OUT_OF_FRAME_POSE = [[10,10], [10, -10], [-10, -10], [-10, 10], [5, 5]]

y_grid = [-0.15, 0, 0.15, 0.3]
x_grid = [0.2, 0.35, 0.5, 0.65]
xy_grid = list(product(x_grid, y_grid))

def _on_robot_state(msg):
    """
        Callback function for updating joint_states
    """
    global CURRENT_POSITION
    CURRENT_POSITION = msg.position[2:]


def move_to_neutral():
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

    while np.linalg.norm(np.array(CURRENT_POSITION) - np.array(neutral_position)) > 10**-2:
        joint_command_publisher.publish(command_msg)
        rate.sleep()

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


def new_scene(POSE_INFO):
    global xy_grid
    in_frame = np.random.randint(low=0, high=5, size=7)
    euler_z_angle = np.random.random(size=7) * 2*np.pi
    for i, name in enumerate(POSE_INFO.name):
        if name == 'ground_plane' or name == 'panda':
            continue
        print(i, name)
        if in_frame[i] == 0:
            state_msg.pose.position.x = OUT_OF_FRAME_POSE[i][0]
            state_msg.pose.position.y = OUT_OF_FRAME_POSE[i][1]
            state_msg.pose.position.z = 0.01
            state_msg.pose.orientation.x = 0
            state_msg.pose.orientation.y = 0
            state_msg.pose.orientation.z = 0
            state_msg.pose.orientation.w = 0
        else:
            xy = random.choice(xy_grid)
            xy_grid.pop(xy_grid.index(xy))
            r = R.from_euler('xyz', [0, 0, euler_z_angle[i]])
            orientation = r.as_quat()
            state_msg = ModelState()
            state_msg.model_name = name
            state_msg.pose.position.x = xy[0]
            state_msg.pose.position.y = xy[1]
            state_msg.pose.position.z = 0.01
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

def model_pose_tracker(data):
    global POSE_INFO
    POSE_INFO = data

        


if __name__ == '__main__':

    rospy.init_node('elsa_perception_benchmark')
    rospy.Subscriber("/gazebo/model_states", gazebo_msgs.msg.ModelStates, queue_size=1, callback=model_pose_tracker)

    scene = 0  

    # move panda to neutral
    try:
        move_to_neutral()
    except rospy.ROSInterruptException:
        pass 


    #model spawner
    model_spawner = rospy.ServiceProxy('/gazebo/spawn_urdf_model', SpawnModel)
    model_spawner.wait_for_service()

    rospy.loginfo('Perception Benchmarking with %d scenarios' % (FLAGS.scenes))
    #spawn_scene()

    # evaluation loop
    while not rospy.is_shutdown() and scene < FLAGS.scenes:
        rospy.sleep(2.0)
        rospy.logwarn('---------------------')
        rospy.loginfo('Scene: {0}'.format(scene+1))
        new_scene(POSE_INFO)
        scene+=1
        rospy.loginfo('Scene over')
    

    rospy.loginfo('Done.')