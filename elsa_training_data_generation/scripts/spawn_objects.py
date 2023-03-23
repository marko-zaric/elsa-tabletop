import rospy
from rospkg import RosPack
import xacro
from geometry_msgs.msg import Pose
from gazebo_msgs.srv import SpawnModel, DeleteModel
from gazebo_msgs.msg import ModelStates
from elsa_perception_msgs.msg import PhysicalScene

import argparse
import numpy as np
from copy import deepcopy
import random
from os.path import join
import threading

def euler_to_quaternion(yaw, pitch, roll):
    qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
    qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
    qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    return qx, qy, qz, qw


class DataGen:
    def __init__(self):

        # cmd arguments
        parser = argparse.ArgumentParser()
        parser.add_argument('--episodes', type=int, default=1, help='Number of episodes to run')
        parser.add_argument('--model', type=str, default='cylinder', help='Set object') # all_models = ['cylinder', 'box', 'sphere']
        self.FLAGS = parser.parse_args()
        

        self.pose = Pose()
        self.scene = PhysicalScene()

        rospy.init_node('elsa_training_data_generator')
        self.pose_lock = threading.Lock()  

        #gazebo
        self.model_spawner = rospy.ServiceProxy('/gazebo/spawn_urdf_model', SpawnModel)
        self.model_remover = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)
        self.model_spawner.wait_for_service()
        self.model_remover.wait_for_service()
        self.model_pose_tracker = rospy.Subscriber('/gazebo/model_states_throttled', ModelStates, queue_size=1, callback=self.model_pose_callback, tcp_nodelay=True)
        self.scene_tracker = rospy.Subscriber('/scene_description', PhysicalScene, self.scene_callback)
        #rospack
        self.rp = RosPack()
        self.SCENE_OBSERVED = False
        self.X = np.ones((132))
        


    def randomize_model(self, model_urdf_xacro):
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
                multiplicator = np.random.uniform(1., 3.)
                model_xacro_args[arg] = str(float(arg_val)*multiplicator)

        # Process the file once more using the new args
        new_opts = vars(opts)
        new_opts['mappings'] = model_xacro_args
        model_xacro = xacro.process_file(model_urdf_xacro, **new_opts)
        # Dump to xml and return
        return model_xacro.toxml(), new_opts

    def randomize_model_pose(self):
        translator_x = np.random.uniform(-0.05, 0.05)
        translator_y = np.random.uniform(-0.05, 0.05)
        self.pose.position.x = np.random.uniform(0.3, 0.7) + translator_x
        self.pose.position.y = np.random.uniform(-0.6, 0.6) + translator_y
        self.pose.position.z = 1.3
        angle = np.random.uniform(0.0, np.pi)
        qx, qy, qz, qw = euler_to_quaternion(angle, 0.0, 0.0)
        self.pose.orientation.w = qw    #1.0
        self.pose.orientation.x = qx    #0.0
        self.pose.orientation.y = qy    #0.0
        self.pose.orientation.z = qz    #0.0

    def spawn_model(self):
        self.current_model = self.FLAGS.model
        rospy.loginfo('Preparing to spawn model: {0}'.format(self.current_model))
        model_urdf_xacro = join(self.rp.get_path('elsa_training_data_generation'), 'models/objects', self.current_model, 'model.urdf.xacro')
        self.randomize_model_pose()
        model_xml, opts = self.randomize_model(model_urdf_xacro)
        try:
            self.model_spawner(self.current_model, model_xml, "/", self.pose, 'world')
        except rospy.ServiceException as e:
            rospy.logerr('Spawn model service call failed: {0}'.format(e))

        rospy.loginfo('Model spawned')
        rospy.sleep(1)
        return opts

    def delete_model(self):
        if self.current_model is None:
            return
        rospy.loginfo('Preparing to delete model: {0}'.format(self.current_model))
        try:
            while True:
                resp_delete = self.model_remover(self.current_model)
                rospy.loginfo(resp_delete.status_message)
                if resp_delete.success == True:
                    break
                rospy.sleep(1.0)            
        except rospy.ServiceException as e:
            rospy.logerr('Delete model service call failed: {0}'.format(e))

        rospy.loginfo('Model deleted')

    def model_pose_callback(self, msg):
        if self.current_model in msg.name:
            idx = msg.name.index(self.current_model)
            with self.pose_lock:
                self.pose = msg.pose[idx]

    def scene_callback(self, msg):
        self.scene = msg.physical_scene
        self.SCENE_OBSERVED = True

    def generate_episode(self, episode):
        opts = self.spawn_model()
        self.SCENE_OBSERVED = False
        while self.SCENE_OBSERVED != True:
            pass
        if len(self.scene) == 1:
            obj = self.scene[0]
            feature_vector = []
            feature_vector.append(obj.spatial_features.x)
            feature_vector.append(obj.spatial_features.y)
            feature_vector.append(obj.spatial_features.z)
            feature_vector.append(obj.spatial_features.phi)
            feature_vector.append(obj.spatial_features.dx)
            feature_vector.append(obj.spatial_features.dy)
            feature_vector.append(obj.spatial_features.dz)
            feature_vector.append(obj.surface_features.normal_azimut.min)
            feature_vector.append(obj.surface_features.normal_azimut.max)
            feature_vector.append(obj.surface_features.normal_azimut.avg)
            feature_vector.append(obj.surface_features.normal_azimut.var)
            feature_vector.append(obj.surface_features.normal_azimut.dev)
            feature_vector = feature_vector + list(obj.surface_features.normal_azimut.his)
            feature_vector.append(obj.surface_features.normal_zenith.min)
            feature_vector.append(obj.surface_features.normal_zenith.max)
            feature_vector.append(obj.surface_features.normal_zenith.avg)
            feature_vector.append(obj.surface_features.normal_zenith.var)
            feature_vector.append(obj.surface_features.normal_zenith.dev)
            feature_vector = feature_vector + list(obj.surface_features.normal_zenith.his)
            feature_vector.append(obj.surface_features.min_curvature.min)
            feature_vector.append(obj.surface_features.min_curvature.max)
            feature_vector.append(obj.surface_features.min_curvature.avg)
            feature_vector.append(obj.surface_features.min_curvature.var)
            feature_vector.append(obj.surface_features.min_curvature.dev)
            feature_vector = feature_vector + list(obj.surface_features.min_curvature.his)
            feature_vector.append(obj.surface_features.max_curvature.min)
            feature_vector.append(obj.surface_features.max_curvature.max)
            feature_vector.append(obj.surface_features.max_curvature.avg)
            feature_vector.append(obj.surface_features.max_curvature.var)
            feature_vector.append(obj.surface_features.max_curvature.dev)
            feature_vector = feature_vector + list(obj.surface_features.max_curvature.his)
            feature_vector.append(obj.surface_features.shape_index.min)
            feature_vector.append(obj.surface_features.shape_index.max)
            feature_vector.append(obj.surface_features.shape_index.avg)
            feature_vector.append(obj.surface_features.shape_index.var)
            feature_vector.append(obj.surface_features.shape_index.dev)
            feature_vector = feature_vector + list(obj.surface_features.shape_index.his)
            self.X = np.vstack((self.X, np.array(feature_vector)))
        else: 
            print("To many objects detected!: ", len(self.scene))
        self.delete_model()


if __name__ == '__main__':
    data_generator = DataGen()
    episode = 1 
    rospy.loginfo('Doing %d episodes' % (data_generator.FLAGS.episodes))

    while not rospy.is_shutdown() and episode <= data_generator.FLAGS.episodes:
        rospy.sleep(1.0)
        rospy.logwarn('---------------------')
        rospy.loginfo('Episode: {0}'.format(episode))
        data_generator.generate_episode(episode)
        episode+=1
        rospy.loginfo('Episode over')

    data_generator.X = np.delete(data_generator.X, obj=0, axis=0)
    rp = RosPack()
    np.save(join(rp.get_path('elsa_training_data_generation'), "data", data_generator.current_model) + "2.npy", data_generator.X) 
    rospy.loginfo('Done.')
