import rospy
import sys
sys.path.append("/home/marko/Desktop/IIS_Research/catkin_workspaces/panda_catkin_ws/src/panda_simulator/iis_panda_controls/scripts")
from pointcloud_bb import pointcloudBB
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
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState
from scipy.spatial.transform import Rotation as R
import numpy as np
from itertools import product
import random
import copy
from sensor_msgs.point_cloud2 import PointCloud2
import sensor_msgs.point_cloud2 as pc2
import time
import ctypes
import struct


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
BOUNDING_BOXES = None
COMPUTATION_TIME = None

OBJ_DIMENSIONS = {'can1':[0.06701,0.06701, 0,1239],
                'can2':[0.06701,0.06701, 0,1239],
                'brick1':[0.06, 0.12, 0.07],
                'brick2':[0.06, 0.12, 0.07],
                'wood_cube_5cm':[0.05, 0.05, 0.05],
                'wood_cube_7_5cm':[0.075, 0.075, 0.075]}


OUT_OF_FRAME_POSE = [[2.0,2.0], [2.0, -2.0], [-2.0, -2.0], [-2.0, 2.0], [2.5, 2.5], [2.0, 2.5]]

y_grid = [-0.15, 0, 0.15, 0.3]
x_grid = [0.2, 0.35, 0.5, 0.65]
XY_GRID = list(product(x_grid, y_grid))

def _on_robot_state(msg):
    """
        Callback function for updating joint_states
    """
    global CURRENT_POSITION
    CURRENT_POSITION = msg.position[2:]


def move_to_neutral():
    
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
        print(np.linalg.norm(np.array(CURRENT_POSITION) - np.array(neutral_position)))
        print(np.array(CURRENT_POSITION))
        print(np.array(neutral_position))
        if np.linalg.norm(np.array(CURRENT_POSITION[:3]) - np.array(neutral_position[:3])) < 10**-2 and np.linalg.norm(np.array(CURRENT_POSITION[4:]) - np.array(neutral_position[4:])) < 10**-2:
            break
        rate.sleep()

def read_camera(data):
    global BOUNDING_BOXES, COMPUTATION_TIME
    gen = pc2.read_points(data, skip_nans=True, field_names=("x", "y", "z", "rgb"))
    count_points = 0
    xyz = np.zeros((data.width, 3))
    rgb = np.zeros((data.width, 3))
    st = time.time()
    for point in gen:
        rgb_float = point[3]
        # # cast float32 to int so that bitwise operations are possible
        s = struct.pack('>f' ,rgb_float)
        i = struct.unpack('>l',s)[0]
        # # you can get back the float value by the inverse operations
        pack = ctypes.c_uint32(i).value
 
        r = (pack & 0x00FF0000)>> 16
        g = (pack & 0x0000FF00)>> 8
        b = (pack & 0x000000FF)

        xyz[count_points, :3] = [point[0], point[1], point[2]]
        rgb[count_points, :3] = [r, g, b]
        count_points += 1

    BOUNDING_BOXES = pointcloudBB(xyz, rgb)
    et = time.time()
    COMPUTATION_TIME = et - st


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

def move_out_of_frame(POSE_INFO):
    out_of_frame_poses = 0
    for i, name in enumerate(POSE_INFO.name):
        if name == 'ground_plane' or name == 'panda':
            continue
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

def new_scene(POSE_INFO):
    xy_grid = copy.copy(XY_GRID)
    in_frame = np.ones(7)#np.random.randint(low=0, high=2, size=7)
    euler_z_angle = np.random.random(size=7) * 2*np.pi
    print("winkel: ", euler_z_angle)
    move_out_of_frame(POSE_INFO)
    for i, name in enumerate(POSE_INFO.name):
        if name == 'ground_plane' or name == 'panda':
            continue
        state_msg = ModelState()
        if in_frame[i] != 0:
            xy = random.choice(xy_grid)
            xy_grid.pop(xy_grid.index(xy))
            r = R.from_euler('xyz', [0, 0, euler_z_angle[i]])
            orientation = r.as_quat()
            state_msg.model_name = name
            print(name, euler_z_angle[i] % np.pi)
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

        
def evaluate(pose_ground_truth, bb_camera, comp_time):
    ary_bbs = np.array(bb_camera)
    
    
    for i, name in enumerate(pose_ground_truth.name):
        if name == 'ground_plane' or name == 'panda':
            continue
        if np.abs(pose_ground_truth.pose[i].position.x) > 1:
            # out of view 
            continue
        print(pose_ground_truth.name[i])
        # print(pose_ground_truth.pose[i])
        # find closest object 
        diff_x = np.abs(ary_bbs[:,:1].T[0] - pose_ground_truth.pose[i].position.x)
        diff_y = np.abs(ary_bbs[:,1:2].T[0] - pose_ground_truth.pose[i].position.y)
        cam_pose = bb_camera[np.argmin(diff_x + diff_y)]
        print(cam_pose[3])
        
        
        


if __name__ == '__main__':

    rospy.init_node('elsa_perception_benchmark')
    rate = rospy.Rate(10)

    # move panda to neutral
    try:
        move_to_neutral()
    except rospy.ROSInterruptException as e:
        rospy.logerr("Could not move to neutral! ", e) 
        rospy.signal_shutdown("Error Shutdown")

    rospy.Subscriber("/gazebo/model_states", gazebo_msgs.msg.ModelStates, queue_size=1, callback=model_pose_tracker)
    rospy.Subscriber("/downsample/output", PointCloud2, callback=read_camera)
    scene = 0 


    #model spawner
    model_spawner = rospy.ServiceProxy('/gazebo/spawn_urdf_model', SpawnModel)
    model_spawner.wait_for_service()

    rospy.loginfo('Perception Benchmarking with %d scenarios' % (FLAGS.scenes))
    #spawn_scene()

    # evaluation loop
    while not rospy.is_shutdown() and scene < FLAGS.scenes:
        rospy.sleep(1)
        rospy.logwarn('---------------------')
        rospy.loginfo('Scene: {0}'.format(scene+1))
        new_scene(POSE_INFO)
        snapshot_pose_info = POSE_INFO
        snapshot_bb = copy.copy(BOUNDING_BOXES)
        snapshot_computation_time = COMPUTATION_TIME
        evaluate(snapshot_pose_info, snapshot_bb, snapshot_computation_time)
        scene+=1
        rospy.loginfo('Scene over')
    

    rospy.loginfo('Done.')