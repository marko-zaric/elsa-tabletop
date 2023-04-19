import rospy
import xacro
from geometry_msgs.msg import Pose
from gazebo_msgs.srv import SpawnModel, DeleteModel, GetWorldProperties
# import argparse
import numpy as np
import random
from os.path import join
from scipy.spatial.transform import Rotation
import numpy as np
from itertools import product
import random
import copy
from elsa_perception_msgs.msg import PhysicalScene
import csv
from datetime import datetime
from rospkg import RosPack
from perception.randomize_sdf import RandomizeSDF
from elsa_perception_msgs.srv import Benchmark, BenchmarkResponse
import pandas as pd
import pprint 
from elsa_object_database.srv import RegisteredObjects

class BenchmarkPerception:
    def __init__(self, number_of_scenes=1, number_of_objects=1, object_type='cube', save_benchmark=False, out_folder='/home/marko/benchmark/'):
        rospy.loginfo(number_of_scenes)
        rospy.loginfo(number_of_objects)
        rospy.loginfo(object_type)
        rospy.loginfo(save_benchmark)
        rospy.loginfo(out_folder)
        self.number_of_scenes = number_of_scenes
        self.number_of_objects = number_of_objects
        self.object = object_type
        self.save_benchmark = save_benchmark
        # benchmark results
        if save_benchmark:
            rospy.loginfo("In")
            datetime_stamp = str(datetime.now())
            self.filename = out_folder + object_type + "_" + datetime_stamp + '.csv'
            self.csv_file = open(self.filename, 'w')
            self.csv_writer = csv.writer(self.csv_file)
            self.csv_writer.writerow(['scene', 'object', 'gt_x', 'gt_y', 'gt_z', 'gt_phi', 'gt_dx', 'gt_dy', 'gt_dz', 'x', 'y', 'z', 'phi', 'dx', 'dy', 'dz', 'gt_round', 'round'])  

        self.scene = None
        self.SCENE_OBSERVED = 0

        self.pose = Pose()
        self.z_euler_angle = None
        self.color = None
        self.bounding_box = None 
        self.is_round = None
        rospy.loginfo("Pose init")
        self.rp = RosPack()
        rospy.loginfo("RosPack init")
        self.sdf_randomizer = RandomizeSDF()
        rospy.loginfo("randomizer init")
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
        registered_objects = get_registered_objs()
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
        for reg_obj in registered_objects.registered_objects:
            self.color_errors[reg_obj.object_name] = copy.deepcopy(self.total_errors)

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
        xy = random.choice(self.XY_GRID_use)
        self.XY_GRID_use.pop(self.XY_GRID_use.index(xy))
        
        self.pose.position.x = xy[0]
        self.pose.position.y = xy[1]
        self.pose.position.z = 1.15

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
        self.color, model_sdf = self.sdf_randomizer.randomize_color(model_sdf)
        self.bounding_box, model_sdf, self.is_round = self.sdf_randomizer.randomize_size(model_sdf)
        
        self.randomize_model_pose()
        try:
            self.model_spawner(self.color, model_sdf, "/", self.pose, 'world')
        except rospy.ServiceException as e:
            rospy.logerr('Spawn model service call failed: {0}'.format(e))
        out_msg = 'Model spawned ' + self.color
        # rospy.loginfo(out_msg)

    def delete_models(self):
        try:
            resp_world = self.gazebo_world()
        except rospy.ServiceException as e:
                rospy.logerr('GeWorldProperties service call failed: {0}'.format(e))
        for name in resp_world.model_names:
            if 'Object' in name:
                try:
                    # while True:
                    resp_delete = self.model_remover(name)
                    # rospy.loginfo(resp_delete.status_message)
                        # if resp_delete.success == True:
                        #     break
                        # rospy.sleep(1.0)            
                except rospy.ServiceException as e:
                    rospy.logerr('Delete model service call failed: {0}'.format(e))
        rospy.loginfo('Models deleted')

    def new_scene(self):
        new_poses = dict()
        self.delete_models()
        self.XY_GRID_use = copy.copy(self.XY_GRID)
        for i in range(self.number_of_objects):
            self.spawn_model(self.object)
            new_poses[self.color] = {"pose": copy.deepcopy(self.pose), 
                               "z_angle":  copy.deepcopy(self.z_euler_angle), 
                            #    "color": copy.deepcopy(self.color),
                               "is_round": copy.deepcopy(self.is_round),
                               "bounding_box": copy.deepcopy(self.bounding_box)}
        self.SCENE_OBSERVED = 0
        return new_poses
        
    def store_data(self, new_poses, scene_count):
        while self.SCENE_OBSERVED < 3:
            rospy.sleep(1.0)
        observed_scene = self.scene
        if len(observed_scene) == 0:
            rospy.logwarn("No objects where observed!")
        else:
            rospy.loginfo("Observed objects: " + str(len(observed_scene)))
            df = pd.DataFrame(columns=['scene', 'object', 'gt_x', 'gt_y', 'gt_z', 'gt_phi', 'gt_dx', 'gt_dy', 'gt_dz', 'x', 'y', 'z', 'phi', 'dx', 'dy', 'dz', 'gt_round', 'round'])
            for obj in observed_scene:
                #color_label = obj.obj_identity.split(" ")[0].lower()
                # Find object in new poses
                GT_object = None
                genarated_object = None
                for gen_obj in new_poses:
                    if obj.obj_identity == gen_obj:
                        genarated_object = gen_obj
                        GT_object = new_poses[gen_obj]

                # check if obj should be round
                print(GT_object)
                gt_round = 0
                print(genarated_object)      
                if GT_object['is_round'] == True:
                    angle = 0.0
                    gt_round = 1
                else:
                    angle = GT_object['z_angle']
                # check if object was percieved as a round object
                round_percept = 0
                if obj.spatial_features.phi == 0.0:
                   round_percept = 1
                
                # select GT (ground truth) dimensions
                # GT_dimensions = None
                # for dims_key in self.OBJ_DIMENSIONS:
                #     if dims_key in genarated_object:
                #         GT_dimensions = self.OBJ_DIMENSIONS[dims_key]
                #         break
                if self.save_benchmark:
                    # write result to csv
                    self.csv_writer.writerow([scene_count, genarated_object, GT_object['pose'].position.x, GT_object['pose'].position.y, GT_object['pose'].position.z,
                                        angle, GT_object['bounding_box'][0], GT_object['bounding_box'][1], 
                                        GT_object['bounding_box'][2], obj.spatial_features.x, obj.spatial_features.y, obj.spatial_features.z,
                                        obj.spatial_features.phi, obj.spatial_features.dx, obj.spatial_features.dy, obj.spatial_features.dz, gt_round, round_percept])   
                else:
                    df.loc[len(df)] = [scene_count, genarated_object, GT_object['pose'].position.x, GT_object['pose'].position.y, GT_object['pose'].position.z,
                                    angle, GT_object['bounding_box'][0], GT_object['bounding_box'][1], 
                                    GT_object['bounding_box'][2], obj.spatial_features.x, obj.spatial_features.y, obj.spatial_features.z,
                                    obj.spatial_features.phi, obj.spatial_features.dx, obj.spatial_features.dy, obj.spatial_features.dz, gt_round, round_percept]
                
                self.update_eval_metrics(genarated_object, GT_object, angle, obj, gt_round, round_percept)
            print(df)

    def update_eval_metrics(self, object_name, gt_obj, gt_angle, obj_percept, gt_round, round_percept):
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
        print("--- Total errors ---")
        self.print_metrics(self.total_errors)
        print("--- Error by quadrant ---")
        for quadrant in self.quadrant_errors:
            print("#", quadrant)
            self.print_metrics(self.quadrant_errors[quadrant])
        print("--- Error by color ---")
        for color in self.color_errors:
            print("#", color)
            self.print_metrics(self.color_errors[color])

    @staticmethod
    def print_metrics(error_dict):
        for error_dimension in error_dict:
            print("----" + error_dimension + "----")
            if error_dict[error_dimension] == []:
                continue
            print("mean:", np.mean(np.array(error_dict[error_dimension])))
            print("std:", np.std(np.array(error_dict[error_dimension])))
            print("max:", np.max(np.array(error_dict[error_dimension])))
            print("min:", np.min(np.array(error_dict[error_dimension])))

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
        rospy.loginfo('Scene: {0}'.format(scene+1))
        benchmark_node.sdf_randomizer = RandomizeSDF()
        GT_poses = benchmark_node.new_scene()
        rospy.loginfo('New scene created')
        benchmark_node.store_data(GT_poses, scene)
        scene += 1
        rospy.loginfo('Scene over')
    if benchmark_node.save_benchmark:
        benchmark_node.csv_file.close()
    benchmark_node.calculate_error_metrics()
    # rospy.loginfo("Total Errors")
    # pp = pprint.PrettyPrinter(indent=2)
    # pp.pprint(benchmark_node.total_errors)
    # rospy.loginfo("Quadrant Errors")
    # pp.pprint(benchmark_node.quadrant_errors)
    # rospy.loginfo("Color Errors")
    # pp.pprint(benchmark_node.color_errors)
    return BenchmarkResponse()


        
def start_benchmarking_server():
    rospy.init_node('elsa_perception_benchmark') 
    rate = rospy.Rate(100)
    service = rospy.Service('perception_benchmark', Benchmark, callback)
    rospy.loginfo("Benchmarking service ready")
    rospy.spin()


if __name__ == '__main__':
    start_benchmarking_server()       


# if __name__ == '__main__':
    
#     benchmark_node = BenchmarkPerception()
#     rospy.loginfo('Perception Benchmarking with %d scenarios' % (benchmark_node.number_of_scenes))
#     # evaluation loop
#     scene = 0
#     while not rospy.is_shutdown() and scene < benchmark_node.number_of_scenes:  
#         rospy.logwarn('---------------------')
#         rospy.loginfo('Scene: {0}'.format(scene+1))
#         benchmark_node.sdf_randomizer = RandomizeSDF()
#         GT_poses = benchmark_node.new_scene()
#         rospy.loginfo('New scene created')
#         benchmark_node.store_data(GT_poses, scene)
#         scene += 1
#         rospy.loginfo('Scene over')
#     benchmark_node.csv_file.close()

#     # Evaluate collected data
#     benchmark_evaluation = BenchmarkEvaluation(benchmark_node.filename)
#     benchmark_evaluation.evaluate()

#     rospy.loginfo('Done.')