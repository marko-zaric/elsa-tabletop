import rospy
from scipy.spatial.transform import Rotation
import pandas as pd 
from gazebo_msgs.srv import SpawnModel
from os.path import join
from perception.sdf_modifier import SDFmodifier
from geometry_msgs.msg import Pose
from rospkg import RosPack
from elsa_benchmark.srv import SpawnScene, SpawnSceneResponse


class SpawnBenchmarkScene:
    def __init__(self):
        self.model_spawner = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
        self.model_spawner.wait_for_service()

        self.sdf_modifier = SDFmodifier()
        self.rp = RosPack()

    
    def spawn_scene(self, file, scene):
        success = True
        self.SCENES = pd.read_csv(file)

        objects = self.SCENES[self.SCENES["scene"] == scene]
        for row, obj in objects.iterrows():
            print(obj['object_color'])
            file_sdf = open(join(self.rp.get_path('elsa_simulator'),'models/benchmark_objects', obj['object_type'], 'model.sdf'))
            model_sdf = file_sdf.read()
            file_sdf.close()
            model_sdf = self.sdf_modifier.set_color(model_sdf, obj['object_color'])
            model_sdf = self.sdf_modifier.set_size(model_sdf, [obj['gt_dx'], obj['gt_dy'], obj['gt_dz']])

            pose = Pose()
            pose.position.x = obj['gt_x']
            pose.position.y = obj['gt_y']
            pose.position.z = 1.15
            
            rotation = Rotation.from_euler('xyz', [0.0, 0.0, obj['gt_phi']])
            orientation = rotation.as_quat()

            pose.orientation.w = orientation[3]    #1.0
            pose.orientation.x = orientation[0]    #0.0
            pose.orientation.y = orientation[1]    #0.0
            pose.orientation.z = orientation[2]    #0.0
            
            try:
                self.model_spawner(obj['object_color'], model_sdf, "/",pose, 'world')
            except rospy.ServiceException as e:
                rospy.logerr('Spawn model service call failed: {0}'.format(e))
                success = False

        return success

    def callback(self, request):
        rospy.loginfo("Clearing objects from scene...")
        success = self.spawn_scene(request.benchmark_file, request.scene)
        return SpawnSceneResponse(success)


def start_spawning_server():
    rospy.init_node('elsa_spawn_scene_server') 
    rate = rospy.Rate(100)
    spawner = SpawnBenchmarkScene()
    service = rospy.Service('benchmark/spawn_specific_scene', SpawnScene, spawner.callback)
    rospy.loginfo("Specific scene spawner service ready!")
    rospy.spin()


if __name__ == '__main__':
    start_spawning_server()       
       

# if __name__ == '__main__':
#     parser = argparse.ArgumentParser()
#     parser.add_argument('--in_file', type=str, help='Benchmark Out file')
#     parser.add_argument('--scene', type=int, help='Scene to look at')
#     FLAGS = parser.parse_args()
#     spawner = SpawnBenchmarkScene(FLAGS.in_file)
#     spawner.spawn_scene(FLAGS.scene)



