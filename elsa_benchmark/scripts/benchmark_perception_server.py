import numpy as np
import random
import rospy
import xacro
from os.path import join
from scipy.spatial.transform import Rotation
import numpy as np
from itertools import product
import copy
import csv
from datetime import datetime
import pandas as pd
import pprint 
from rospkg import RosPack
from perception.sdf_modifier import SDFmodifier
from geometry_msgs.msg import Pose
from gazebo_msgs.srv import SpawnModel, DeleteModel, GetWorldProperties
from elsa_perception_msgs.msg import PhysicalScene
from elsa_benchmark.srv import Benchmark, BenchmarkResponse
from elsa_benchmark.msg import BenchmarkSummary, BenchmarkErrorMetrics, BenchmarkErrorDimension, MaxError
from elsa_object_database.srv import RegisteredObjects

class BenchmarkPerception:
    def __init__(self, number_of_scenes=1, number_of_objects=1, object_type='cube', save_benchmark=False, out_folder='/home/marko/benchmark/'):
        self.number_of_scenes = number_of_scenes
        self.number_of_objects = number_of_objects
        self.object = object_type
        self.save_benchmark = save_benchmark
        self.filename = 'Not stored'

        self.rand = random.Random(4242)
        np.random.seed(1)
        # benchmark results
        if save_benchmark:
            datetime_stamp = str(datetime.now())
            self.filename = out_folder + object_type + "_" + datetime_stamp + '.csv'
            self.csv_file = open(self.filename, 'w')
            self.csv_writer = csv.writer(self.csv_file)
            self.csv_writer.writerow(['scene', 'object_color', 'object_type', 'gt_x', 'gt_y', 'gt_z', 'gt_phi', 'gt_dx', 'gt_dy', 'gt_dz', 'x', 'y', 'z', 'phi', 'dx', 'dy', 'dz', 'gt_round', 'round'])  

        self.scene = None
        self.SCENE_OBSERVED = 0

        self.pose = Pose()
        self.z_euler_angle = None
        self.color = None
        self.bounding_box = None 
        self.is_round = None

        self.rp = RosPack()

        # self.sdf_randomizer = SDFmodifier()

        self.OBJECT_LIST = ["cube", "rectangle", "sphere", "cylinder"]

        # self.OBJ_DIMENSIONS = {'can':[0.06701,0.06701, 0.1239],
        #                 'rectangle':[0.06, 0.12, 0.07],
        #                 'cube_large':[0.075, 0.075, 0.075],
        #                 'cube':[0.05, 0.05, 0.05],
        #                 'sphere':[0.1,0.1,0.1]}

        y_grid = [-0.15, 0, 0.15, 0.3]
        x_grid = [0.2, 0.35, 0.5, 0.65]
        self.XY_GRID = list(product(x_grid, y_grid))
        self.XY_GRID_use = None
        
        
        self.model_spawner = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
        self.model_remover = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)
        self.model_spawner.wait_for_service()
        self.model_remover.wait_for_service()

        self.gazebo_world = rospy.ServiceProxy('/gazebo/get_world_properties', GetWorldProperties)
        self.gazebo_world.wait_for_service()

        self.scene_tracker = rospy.Subscriber("/scene_description",PhysicalScene , queue_size=1, callback=self.read_camera)
        
        rospy.wait_for_service('get_registered_obj_service')
        get_registered_objs = rospy.ServiceProxy("get_registered_obj_service", RegisteredObjects)
        self.registered_objects = get_registered_objs()
        # Eval metrics
        self.total_errors = {'x_error': [],
                             'y_error': [],
                             'dx_error': [],
                             'dy_error': [],
                             'dz_error': [],
                             'phi_error': [],
                             'round_error': []}

        self.quadrant_errors = {'1': copy.deepcopy(self.total_errors),
                                '2': copy.deepcopy(self.total_errors),
                                '3': copy.deepcopy(self.total_errors),
                                '4': copy.deepcopy(self.total_errors)}
        
        self.color_errors = {}
        for reg_obj in self.registered_objects.registered_objects:
            self.color_errors[reg_obj.object_name] = copy.deepcopy(self.total_errors)

        self.total_errors['all_objs_detected'] = []

    def read_camera(self, msg):
        self.scene = msg.physical_scene
        self.SCENE_OBSERVED += 1


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

    def randomize_model_pose(self):
        xy = self.rand.choice(self.XY_GRID_use)
        self.XY_GRID_use.pop(self.XY_GRID_use.index(xy))
        
        self.pose.position.x = xy[0]
        self.pose.position.y = xy[1]
        self.pose.position.z = 1.03

        self.z_euler_angle = np.random.uniform(0.0, np.pi)
        rotation = Rotation.from_euler('xyz', [0.0, 0.0, self.z_euler_angle])
        orientation = rotation.as_quat()

        self.pose.orientation.w = orientation[3]    #1.0
        self.pose.orientation.x = orientation[0]    #0.0
        self.pose.orientation.y = orientation[1]    #0.0
        self.pose.orientation.z = orientation[2]    #0.0


    def spawn_model(self, model_name):
        # rospy.loginfo('Preparing to spawn model: {0}'.format(model_name))
        file_sdf = open(join(self.rp.get_path('elsa_simulator'),'models/benchmark_objects', model_name, 'model.sdf'))
        model_sdf = file_sdf.read()
        file_sdf.close()
        self.color, model_sdf = self.sdf_randomizer.randomize_color(model_sdf, self.rand)
        self.bounding_box, model_sdf, self.is_round = self.sdf_randomizer.randomize_size(model_sdf, self.rand)
        
        self.randomize_model_pose()
        try:
            self.model_spawner(self.color, model_sdf, "/", self.pose, 'world')
        except rospy.ServiceException as e:
            rospy.logerr('Spawn model service call failed: {0}'.format(e))
        out_msg = 'Model spawned ' + self.color
        rospy.loginfo(out_msg)

    def delete_models(self):
        try:
            resp_world = self.gazebo_world()
        except rospy.ServiceException as e:
                rospy.logerr('GeWorldProperties service call failed: {0}'.format(e))
        for name in resp_world.model_names:
            if 'Object' in name:
                try:
                    resp_delete = self.model_remover(name)          
                except rospy.ServiceException as e:
                    rospy.logerr('Delete model service call failed: {0}'.format(e))
        rospy.loginfo('Models deleted')

    def new_scene(self):
        new_poses = dict()
        self.delete_models()
        rospy.sleep(1)
        self.XY_GRID_use = copy.copy(self.XY_GRID)
        for i in range(self.number_of_objects):
            self.spawn_model(self.object)
            new_poses[self.color] = {"pose": copy.deepcopy(self.pose), 
                                "z_angle":  copy.deepcopy(self.z_euler_angle), 
                                "object_type": copy.deepcopy(self.object),
                                "is_round": copy.deepcopy(self.is_round),
                                "bounding_box": copy.deepcopy(self.bounding_box)}
        self.SCENE_OBSERVED = 0
        return new_poses
        
    def store_data(self, new_poses, scene_count):
        while self.SCENE_OBSERVED < 3:
            rospy.sleep(1.0)
        observed_scene = self.scene
        all_objects_detected = True
        if len(observed_scene) == 0:
            rospy.logwarn("No objects where observed!")
        else:
            rospy.loginfo("Observed objects: " + str(len(observed_scene)))
            df = pd.DataFrame(columns=['scene', 'object_color', 'object_type', 'gt_x', 'gt_y', 'gt_z', 'gt_phi', 'gt_dx', 'gt_dy', 'gt_dz', 'x', 'y', 'z', 'phi', 'dx', 'dy', 'dz', 'gt_round', 'round'])
            for genarated_object in new_poses:
                # Match generated object with percived object by color identity
                perceived_object = None
                GT_object = new_poses[genarated_object]
                for obj in observed_scene:
                    
                    print(obj.obj_identity)

                    if obj.obj_identity == genarated_object:
                        perceived_object = obj

                # check if obj should be round
                    gt_round = 0     
                    if GT_object['is_round'] == True:
                        angle = 0.0
                        gt_round = 1
                    else:
                        angle = GT_object['z_angle']
                    round_percept = ''

                if perceived_object is None:
                    all_objects_detected = False
                    if self.save_benchmark:
                        # write result to csv
                        self.csv_writer.writerow([scene_count, genarated_object, GT_object['object_type'], GT_object['pose'].position.x, GT_object['pose'].position.y, GT_object['pose'].position.z,
                            angle, GT_object['bounding_box'][0], GT_object['bounding_box'][1], 
                                            GT_object['bounding_box'][2], '', '', '', '', '', '', '', gt_round, ''])   
                    else:
                        df.loc[len(df)] = [scene_count, genarated_object, GT_object['object_type'], GT_object['pose'].position.x, GT_object['pose'].position.y, GT_object['pose'].position.z,
                                        angle, GT_object['bounding_box'][0], GT_object['bounding_box'][1], 
                                        GT_object['bounding_box'][2], '', '', '', '', '', '', '', gt_round, '']
                else:
                    
                    # check if object was percieved as a round object
                    round_percept = 0
                    if perceived_object.spatial_features.phi == 0.0:
                       round_percept = 1

                    if self.save_benchmark:
                        # write result to csv
                        self.csv_writer.writerow([scene_count, genarated_object, GT_object['object_type'], GT_object['pose'].position.x, GT_object['pose'].position.y, GT_object['pose'].position.z,
                                            angle, GT_object['bounding_box'][0], GT_object['bounding_box'][1], 
                                            GT_object['bounding_box'][2], perceived_object.spatial_features.x, perceived_object.spatial_features.y, perceived_object.spatial_features.z,
                                            perceived_object.spatial_features.phi, perceived_object.spatial_features.dx, perceived_object.spatial_features.dy, perceived_object.spatial_features.dz, gt_round, round_percept])   
                    else:
                        df.loc[len(df)] = [scene_count, genarated_object, GT_object['object_type'], GT_object['pose'].position.x, GT_object['pose'].position.y, GT_object['pose'].position.z,
                                        angle, GT_object['bounding_box'][0], GT_object['bounding_box'][1], 
                                        GT_object['bounding_box'][2], obj.spatial_features.x, obj.spatial_features.y, obj.spatial_features.z,
                                        obj.spatial_features.phi, obj.spatial_features.dx, obj.spatial_features.dy, obj.spatial_features.dz, gt_round, round_percept]
                
                self.update_eval_metrics(genarated_object, GT_object, angle, perceived_object, gt_round, round_percept, all_objects_detected)
                if all_objects_detected == True:
                    self.total_errors['all_objs_detected'].append(True)
                else:
                    self.total_errors['all_objs_detected'].append(False)
            print(df)

    def update_eval_metrics(self, object_name, gt_obj, gt_angle, obj_percept, gt_round, round_percept, all_objects_detected):
        self.add_to_error_dict(self.total_errors, gt_obj, gt_angle, obj_percept, gt_round, round_percept)

        # Add to correct quadrant
        if gt_obj['pose'].position.x -0.5 >= 0.0 and gt_obj['pose'].position.y >= 0.0:
            self.add_to_error_dict(self.quadrant_errors['1'], gt_obj, gt_angle, obj_percept, gt_round, round_percept)
        elif gt_obj['pose'].position.x -0.5 < 0.0 and gt_obj['pose'].position.y >= 0.0:
            self.add_to_error_dict(self.quadrant_errors['2'], gt_obj, gt_angle, obj_percept, gt_round, round_percept)
        elif gt_obj['pose'].position.x -0.5 < 0.0 and gt_obj['pose'].position.y < 0.0:
            self.add_to_error_dict(self.quadrant_errors['3'], gt_obj, gt_angle, obj_percept, gt_round, round_percept)
        else:
            self.add_to_error_dict(self.quadrant_errors['4'], gt_obj, gt_angle, obj_percept, gt_round, round_percept)

        # Add to correct color
        self.add_to_error_dict(self.color_errors[object_name], gt_obj, gt_angle, obj_percept, gt_round, round_percept)

    def add_to_error_dict(self, error_dict, gt_obj, gt_angle, obj_percept, gt_round, round_percept):
        if obj_percept is None:
            return
        
        error_dict['x_error'].append(gt_obj['pose'].position.x - obj_percept.spatial_features.x)
        error_dict['y_error'].append(gt_obj['pose'].position.y - obj_percept.spatial_features.y)
        
        correct_orientation = True
        dx_1_error = gt_obj['bounding_box'][0] - obj_percept.spatial_features.dx
        dx_2_error = gt_obj['bounding_box'][0] - obj_percept.spatial_features.dy

        if abs(dx_1_error) > abs(dx_2_error):
            correct_orientation = False
            error_dict['dx_error'].append(gt_obj['bounding_box'][0] - obj_percept.spatial_features.dy)
            error_dict['dy_error'].append(gt_obj['bounding_box'][1] - obj_percept.spatial_features.dx)
        else:
            error_dict['dx_error'].append(gt_obj['bounding_box'][0] - obj_percept.spatial_features.dx)
            error_dict['dy_error'].append(gt_obj['bounding_box'][1] - obj_percept.spatial_features.dy)
        
        error_dict['dz_error'].append(gt_obj['bounding_box'][2] - obj_percept.spatial_features.dz)
        error_dict['phi_error'].append((gt_angle % (np.pi/2)) - (obj_percept.spatial_features.phi % (np.pi/2)))

        error_dict['round_error'].append(gt_round-round_percept)

    def calculate_error_metrics(self):
        benchmark_summary = BenchmarkSummary()
        benchmark_summary.worst_scenes_per_dimension = self.worst_error_per_dimension(self.total_errors)
        benchmark_summary.total_error = self.get_error_metrics(self.total_errors,'total_error')
        benchmark_summary.all_objects_detected = self.total_errors['all_objs_detected']
        benchmark_summary.quadrant_errors = []
        for quadrant in self.quadrant_errors:
            benchmark_summary.quadrant_errors.append(self.get_error_metrics(self.quadrant_errors[quadrant],"Quadrant "+quadrant))
        benchmark_summary.color_errors = []
        for color in self.color_errors:
            benchmark_summary.color_errors.append(self.get_error_metrics(self.color_errors[color], color))
        benchmark_summary.out_file = self.filename
        return benchmark_summary

    @staticmethod
    def get_error_metrics(error_dict, domain_name):
        error_metrics = BenchmarkErrorMetrics()
        error_metrics.domain_name = domain_name
        error_metrics.error_dimensions = []
        error_metrics.reached_in_benchmark = True
        for error_dimension in error_dict:
            error_dim_msg = BenchmarkErrorDimension()
            error_dim_msg.error_dimension = error_dimension
            if error_dict[error_dimension] == []:
                error_metrics.reached_in_benchmark = False
                break
            error_dim_msg.mean = np.mean(np.array(error_dict[error_dimension]))
            error_dim_msg.std = np.std(np.array(error_dict[error_dimension]))
            error_dim_msg.min = np.min(np.array(error_dict[error_dimension]))
            error_dim_msg.max = np.max(np.array(error_dict[error_dimension]))
            error_metrics.error_dimensions.append(error_dim_msg)
        return error_metrics
    
    @staticmethod
    def worst_error_per_dimension(error_dict):
        max_errors = []
        for key in error_dict:
            if key == 'all_objs_detected':
                continue
            max_error_msg = MaxError()
            max_error_msg.dimension = key 
            idx_max_error = np.argmax(np.abs(error_dict[key]))
            max_error_msg.error_value = error_dict[key][idx_max_error]
            max_error_msg.scene = idx_max_error + 1 #returns the scene with the highest error value
            max_errors.append(max_error_msg)
        return max_errors


def callback(request):
    benchmark_node = BenchmarkPerception(request.number_of_scenes, 
                                         request.number_of_objects, 
                                         request.object_type,
                                         request.save_benchmark,
                                         request.out_folder)
    
    rospy.loginfo('Perception Benchmarking with %d scenarios' % (benchmark_node.number_of_scenes))
    # evaluation loop
    scene = 0
    while not rospy.is_shutdown() and scene < benchmark_node.number_of_scenes:  
        rospy.logwarn('---------------------')
        rospy.logwarn('Scene: {0}'.format(scene + 1))
        benchmark_node.sdf_randomizer = SDFmodifier(benchmark_node.registered_objects)
        GT_poses = benchmark_node.new_scene()
        rospy.sleep(2)
        rospy.loginfo('New scene created')
        benchmark_node.store_data(GT_poses, scene+1)
        scene += 1
        rospy.loginfo('Scene over')
    if benchmark_node.save_benchmark:
        benchmark_node.csv_file.close()
    benchmark_summary = BenchmarkSummary()
    benchmark_summary = benchmark_node.calculate_error_metrics()
    rospy.loginfo("All {0} scenes are done.".format(scene + 1))
    return BenchmarkResponse(benchmark_summary)


        
def start_benchmarking_server():
    rospy.init_node('elsa_perception_benchmark') 
    rate = rospy.Rate(100)
    service = rospy.Service('benchmark/perception_benchmark', Benchmark, callback)
    rospy.loginfo("Benchmarking service ready")
    rospy.spin()


if __name__ == '__main__':
    start_benchmarking_server()       