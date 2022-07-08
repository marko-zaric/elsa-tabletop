import rospy
from rospkg import RosPack
import xacro
from geometry_msgs.msg import Pose, PointStamped, Point, WrenchStamped
from std_srvs.srv import Empty, SetBool
from gazebo_msgs.srv import SpawnModel, DeleteModel, SetModelState, SetModelStateRequest, ApplyBodyWrench, ApplyBodyWrenchRequest, BodyRequest, BodyRequestRequest
from gazebo_msgs.msg import ModelStates
from visualization_msgs.msg import Marker
from sensor_msgs.msg import JointState, Image
from std_msgs.msg import String, Bool, Int32

import sys
import argparse
import numpy as np
from copy import deepcopy
import logging
import traceback
import random
from os.path import join
import threading
import json
from poisson_disc import Grid


# cmd arguments
parser = argparse.ArgumentParser()
parser.add_argument('--episodes', type=int, default=1, help='Number of episodes to run')
parser.add_argument('--trials', type=int, default=100, help='Number trials per episode')
parser.add_argument('--episode', type=int, default=1, help='Set current episode')
parser.add_argument('--duration', type=float, default=5.0, help='Duration of force application (or one trial)')
FLAGS = parser.parse_args()

# globals
#all_models = ['cylinder', 'box', 'sphere']
#all_models = ['cylinder', 'box']
all_models = ['cylinder']
#all_models = ['sphere']
pose = Pose()
pose_lock = None
model_spawner = None
model_remover = None
current_model = None
recorder = None
reset_recorder = None
object_notifier = None
episode_notifier = None
trial_notifier = None
obj_info_processor = None
contact_info_processor = None
force_info_processor = None
duration_info_processor = None
model_setter = None
body_setter = None
rp = None
waiter = None
wait = True
object_at_rest = True


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

    current_model = random.choice(all_models)
    object_notifier.publish(current_model)
    rospy.loginfo('Preparing to spawn model: {0}'.format(current_model))
    model_urdf_xacro = join(rp.get_path('iiwanhoe_description'), 'models/objects', current_model, 'model.urdf.xacro')
    randomize_model_pose()
    model_xml, opts = randomize_model(model_urdf_xacro)

    try:
        model_spawner(current_model, model_xml, "/", pose, 'world')
    except rospy.ServiceException as e:
        rospy.logerr('Spawn model service call failed: {0}'.format(e))

    rospy.loginfo('Model spawned')

    #pose update
    rospy.loginfo('Awaiting pose update')
    rospy.sleep(10.0)

    obj_info_processor.publish(model_xml)

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


def compute_application_point(opts):
    p = Point()
    q = Point()
    n_samples = 500

    if current_model == 'sphere':
        radius = float(opts['mappings']['radius'])
        d = 3 # or any positive integer
        points = np.random.normal(size=(n_samples, d)) 
        points /= np.linalg.norm(points, axis=1)[:, np.newaxis]
        # Scale to radius
        points = radius * points        
        upper_z = radius/2.
        lower_z = -radius/2.
        allowed_points = [x for x in points if lower_z <= x[2] <= upper_z]
        idx = np.random.choice(len(allowed_points), size=2)
        p.x = allowed_points[idx[0]][0]
        p.y = allowed_points[idx[0]][1]
        p.z = allowed_points[idx[0]][2]
        q.x = allowed_points[idx[1]][0]
        q.y = allowed_points[idx[1]][1]
        q.z = allowed_points[idx[1]][2]
    elif current_model == 'box':
        width = float(opts['mappings']['width'])        
        depth = float(opts['mappings']['depth'])
        height = float(opts['mappings']['height'])
        w_range = [(-width/2.0), (width/2.0)]
        h_range = [(-height/2.0+0.02), (height/2.0-0.02)]
        d_range = [(-depth/2.0), (depth/2.0)]
        hxd_area = np.abs(h_range[1]-h_range[0]) * np.abs(d_range[1]-d_range[0])
        wxd_area = np.abs(w_range[1]-w_range[0]) * np.abs(d_range[1]-d_range[0])
        wxh_area = np.abs(w_range[1]-w_range[0]) * np.abs(h_range[1]-h_range[0])    
    
        # Calculate side distribution sizes relative to surface areas
        total_area = 2*hxd_area + 2*wxd_area + 2*wxh_area
        n_hxd_side = int(np.round(n_samples * hxd_area/total_area))
        n_wxd_side = int(np.round(n_samples * wxd_area/total_area))
        n_wxh_side = int(np.round(n_samples * wxh_area/total_area))
    
        # Sample side surfaces
        hxd_1_x = w_range[0]*np.ones(n_hxd_side)
        hxd_1_y = np.random.uniform(low=h_range[0], high=h_range[1], size=n_hxd_side)
        hxd_1_z = np.random.uniform(low=d_range[0], high=d_range[1], size=n_hxd_side)
        hxd_2_x = w_range[1]*np.ones(n_hxd_side)
        hxd_2_y = np.random.uniform(low=h_range[0], high=h_range[1], size=n_hxd_side)
        hxd_2_z = np.random.uniform(low=d_range[0], high=d_range[1], size=n_hxd_side)
    
        wxd_1_x = np.random.uniform(low=w_range[0], high=w_range[1], size=n_wxd_side)
        wxd_1_y = h_range[0]*np.ones(n_wxd_side)
        wxd_1_z = np.random.uniform(low=d_range[0], high=d_range[1], size=n_wxd_side)
        wxd_2_x = np.random.uniform(low=w_range[0], high=w_range[1], size=n_wxd_side)
        wxd_2_y = h_range[1]*np.ones(n_wxd_side)
        wxd_2_z = np.random.uniform(low=d_range[0], high=d_range[1], size=n_wxd_side)
    
        wxh_1_x = np.random.uniform(low=w_range[0], high=w_range[1], size=n_wxh_side)
        wxh_1_y = np.random.uniform(low=h_range[0], high=h_range[1], size=n_wxh_side)
        wxh_1_z = d_range[0]*np.ones(n_wxh_side)
        wxh_2_x = np.random.uniform(low=w_range[0], high=w_range[1], size=n_wxh_side)
        wxh_2_y = np.random.uniform(low=h_range[0], high=h_range[1], size=n_wxh_side)
        wxh_2_z = d_range[1]*np.ones(n_wxh_side)
    
        # Concatenate sample groups
        x = np.concatenate([hxd_1_x, hxd_2_x, wxd_1_x, wxd_2_x, wxh_1_x, wxh_2_x])
        y = np.concatenate([hxd_1_y, hxd_2_y, wxd_1_y, wxd_2_y, wxh_1_y, wxh_2_y])
        z = np.concatenate([hxd_1_z, hxd_2_z, wxd_1_z, wxd_2_z, wxh_1_z, wxh_2_z])
        
        # Create final array
        points = np.stack([x,y,z]).transpose()
        p_ = random.choice(points)
        p.x = p_[0]
        p.y = p_[1]
        p.z = p_[2]
        q_cand = [x for x in points if not np.allclose(np.array(p_), np.array(x))]
        q_ = random.choice(q_cand)
        q.x = q_[0]
        q.y = q_[1]
        q.z = q_[2]
    elif current_model == "cylinder":
        radius = float(opts['mappings']['radius'])
        height = float(opts['mappings']['length'])
        phi = np.random.uniform(0.01, np.pi-0.01)
        p.x = radius * np.cos(phi)
        p.y = radius * np.sin(phi)
        p.z = random.uniform(0.02, height-0.02)        
        phi = np.random.uniform(np.pi+0.01, 2*np.pi-0.01)
        q.x = radius * np.cos(phi)
        q.y = radius * np.sin(phi)
        q.z = random.uniform(0.02, height-0.02)        
    else:
        raise('Unknown model: ' + current_model + '!')

    rospy.loginfo('Computed application point...')

    contact_info_processor.publish(str(p))

    return p, q


def compute_force_vector(opts):
    mass = float(opts['mappings']['mass'])

    msg = ApplyBodyWrenchRequest()
    msg.body_name = current_model + '::' + current_model
    msg.reference_frame = current_model + '::' + current_model
    #msg.reference_frame = 'world'

    p, q = compute_application_point(opts)
    h = (p.z+q.z)/.5
    p_ = np.array([p.x, p.y, h])
    q_ = np.array([q.x, q.y, h])
    msg.reference_point = p
    msg.reference_point.z = h

    # a bit of physics:
    # mass is in kg (from urdf)
    # ApplyBodyWrench needs a force (assuming Newtons)
    # static dry friction apply when object are still
    # We must overcome F_max = mu * F_normal = mu * gravity
    # dynamic dry friction apply when surfaces are moving
    # amplitude of static friction to overcome:
    Fmax = 0.4 * mass * 9.8
    # We compute a reasonable variation around the minimal force:
    mu = 1.1 
    sig = 0.08
    modulation = np.random.normal(mu, sig)
    rospy.logerr('Gaussian modulation : %f', modulation)
    rospy.logwarn('mass: %f', mass)


    f = (q_-p_)
    f = f/np.linalg.norm(f)
    rospy.logwarn('direction: %s', f)
    rospy.logwarn('Fmax: %s', Fmax)
    
    # Sphere actually suffer almost no friction,
    # so let's try with a very small range of force
    if current_model == 'sphere':
        # sphere modulation
        f *= 0.0001 * modulation
    elif current_model == 'cylinder':
        # cylinder modulation
        f *= 0.05 #* modulation
    else: 
        # cube modulation
        # not using Fmax at the moment as this
        # factor offer a reasonable motion
        f *= 6.5 * modulation

    #f *= Fmax * modulation
    rospy.logwarn('force: %s', f)

    '''
    #modulate forces
    x_modulator = 1.0
    y_modulator = 1.0
    #z_modulator = 1.0
    if current_model == 'sphere':
        x_modulator = random.uniform(0.005, 0.015)
        y_modulator = random.uniform(0.005, 0.015)
        #z_modulator = random.uniform(0.01, 0.05)
    elif current_model == 'cylinder':
        x_modulator = random.uniform(0.85, 0.95)
        y_modulator = random.uniform(0.85, 0.95)
        #z_modulator = random.uniform(1.05, 1.15)
    else:
        x_modulator = random.uniform(1.15, 1.25)
        y_modulator = random.uniform(1.15, 1.25)
        #z_modulator = random.uniform(1.15, 1.25)

    rospy.logwarn('x_mod %f', x_modulator)
    rospy.logwarn('y_mod %f', y_modulator)
   
    msg.wrench.force.x = f[0] * x_modulator
    msg.wrench.force.y = f[1] * y_modulator
    #msg.wrench.force.z = f[2] * z_modulator
    '''
    # projecting on components
    msg.wrench.force.x = f[0]
    msg.wrench.force.y = f[1]
    msg.wrench.force.z = f[2]

    rospy.logwarn('final x %f', msg.wrench.force.x)
    rospy.logwarn('final_y %f', msg.wrench.force.y)
    rospy.logwarn('final_z %f', msg.wrench.force.z)
    rospy.logwarn('application point: %s', [msg.reference_point.x, msg.reference_point.y, msg.reference_point.z])

    msg.wrench.torque.x = 0.0
    msg.wrench.torque.y = 0.0
    msg.wrench.torque.z = 0.0

    msg.start_time.secs = 0.0
    msg.start_time.nsecs = 0.0
    msg.duration.secs = -1
    msg.duration.nsecs = -1

    wr_msg = WrenchStamped()
    wr_msg.wrench = msg.wrench
    wr_msg.header.frame_id = 'world'
    debug_wrench_pub.publish(wr_msg)

    rospy.loginfo('Computed forces...')

    force_info_processor.publish(str(msg.wrench.force))

    return msg


def object_at_rest_callback(msg):
    global object_at_rest

    if not object_at_rest:
        if current_model in msg.name:
            idx = msg.name.index(current_model)
            twist = msg.twist[idx]
            ang = np.array([twist.angular.x, twist.angular.y, twist.angular.z])          
            lin = np.array([twist.linear.x, twist.linear.y, twist.linear.z])
            if np.allclose(ang, np.array([0.0, 0.0, 0.0]), rtol=0.1, atol=0.1) and np.allclose(lin, np.array([0.0, 0.0, 0.0]), rtol=0.1, atol=0.1):
                object_at_rest = True

            pos = msg.pose[idx].position
            if pos.z < 0.7 or not -0.2 < pos.x < 1.7 or not -0.2 < pos.y < 1.6:
                object_at_rest = True


def stop():
    req = BodyRequestRequest()
    req.body_name = current_model + '::' + current_model
    body_setter(req)

    with pose_lock:
        req = SetModelStateRequest()
        req.model_state.model_name = current_model
        req.model_state.reference_frame = 'world'
        req.model_state.pose = deepcopy(pose)
        req.model_state.twist.linear.x = 0.0
        req.model_state.twist.linear.y = 0.0
        req.model_state.twist.linear.z = 0.0
        req.model_state.twist.angular.x = 0.0
        req.model_state.twist.angular.y = 0.0
        req.model_state.twist.angular.z = 0.0    
        model_setter(req)


def model_pose_callback(msg):
    global pose

    if current_model in msg.name:
        idx = msg.name.index(current_model)
        with pose_lock:
            pose = msg.pose[idx]


def generate_episode(episode):
    global wait, object_at_rest

    trial = 1

    opts = spawn_model()
    episode_notifier.publish(episode)

    while trial <= FLAGS.trials:
        rospy.loginfo('Trial: %d' % trial)
        trial_notifier.publish(trial)
        
        push_force = compute_force_vector(opts)
        duration_info_processor.publish(str(FLAGS.duration))
        recorder(True)
        rospy.loginfo('Pushing!')
        object_at_rest = False
        rospy.loginfo('prepose: %s', [pose.position.x, pose.position.y, pose.position.z])
        forcer(push_force)        
        rospy.sleep(FLAGS.duration)
        rospy.loginfo('Pushing done.')
        recorder(False)        
        stop()
        rospy.loginfo('Deliberate pause...')
        rospy.sleep(10)
        rospy.loginfo('postpose: %s', [pose.position.x, pose.position.y, pose.position.z])
        rospy.loginfo('Over')
        while not object_at_rest:
            rospy.loginfo('Waiting for object to come to halt...')
            rospy.sleep(.5)
        
        rospy.loginfo('Waiting...')
        while wait:
            rospy.sleep(1.0)
        wait = True
        rospy.loginfo('Continuing')  

        with pose_lock:
            if pose.position.z < 0.7 or pose.position.z > 20. or  np.abs(pose.position.x) > 20.0 or np.abs(pose.position.y) > 20.0:
                rospy.loginfo('Object fell of the table. Stopping episode.')
                break
            else:
                trial+=1

    delete_model()


if __name__ == '__main__':

    rospy.init_node('iiwa_episodic_force_pusher')
    pose_lock = threading.Lock()
    episode = FLAGS.episode    

    #gazebo
    model_spawner = rospy.ServiceProxy('/gazebo/spawn_urdf_model', SpawnModel)
    model_remover = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)
    forcer = rospy.ServiceProxy('/gazebo/apply_body_wrench', ApplyBodyWrench)
    model_setter = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
    body_setter = rospy.ServiceProxy('/gazebo/clear_body_wrenches', BodyRequest)
    model_spawner.wait_for_service()
    model_remover.wait_for_service()
    forcer.wait_for_service()
    model_setter.wait_for_service()
    body_setter.wait_for_service()
    model_pose_tracker = rospy.Subscriber('/gazebo/model_states_throttled', ModelStates, queue_size=1, callback=model_pose_callback, tcp_nodelay=True)
    model_pose_tracker = rospy.Subscriber('/gazebo/model_states_throttled', ModelStates, queue_size=1, callback=object_at_rest_callback, tcp_nodelay=True)

      #rospack
    rp = RosPack()

    #recorder
    rospy.wait_for_service('/topic_recorder/record')
    rospy.wait_for_service('/topic_recorder/reset')
    recorder = rospy.ServiceProxy('/topic_recorder/record', SetBool)
    reset_recorder = rospy.ServiceProxy('/topic_recorder/reset', SetBool)
    
    object_notifier = rospy.Publisher('/topic_recorder/object', String, queue_size=1)
    episode_notifier = rospy.Publisher('/topic_recorder/episode', Int32, queue_size=1)
    trial_notifier = rospy.Publisher('/topic_recorder/trial', Int32, queue_size=1)    
    waiter = rospy.Subscriber('/topic_recorder/ready', Bool, waiter_callback)
    obj_info_processor = rospy.Publisher('/topic_recorder/store_obj_info', String, queue_size=1)
    contact_info_processor = rospy.Publisher('/topic_recorder/store_contact_info', String, queue_size=1)
    force_info_processor = rospy.Publisher('/topic_recorder/store_force_info', String, queue_size=1)
    duration_info_processor = rospy.Publisher('/topic_recorder/store_duration_info', String, queue_size=1)
    
    # only for debug:
    debug_app_pt_pub = rospy.Publisher('/application_point', Marker, queue_size=1)
    debug_wrench_pub = rospy.Publisher('/applied_wrench', WrenchStamped, queue_size=1)

    rospy.loginfo('Doing %d episodes with max %d trials each' % (FLAGS.episodes, FLAGS.trials))

    while not rospy.is_shutdown() and episode <= (FLAGS.episodes+FLAGS.episode-1):
        rospy.sleep(5.0)
        rospy.logwarn('---------------------')
        rospy.loginfo('Episode: {0}'.format(episode))
        generate_episode(episode)
        episode+=1
        rospy.loginfo('Episode over')

    rospy.loginfo('Done.')
