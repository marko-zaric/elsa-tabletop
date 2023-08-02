import rospy
from gazebo_msgs.srv import DeleteModel, GetWorldProperties
from elsa_benchmark.srv import ClearObjectsFromScene, ClearObjectsFromSceneResponse

class SceneCleaner:
    def __init__(self):
        self.model_remover = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)
        self.model_remover.wait_for_service()

        self.gazebo_world = rospy.ServiceProxy('/gazebo/get_world_properties', GetWorldProperties)
        self.gazebo_world.wait_for_service()


    def clear_scene(self):
        success = True
        try:
            resp_world = self.gazebo_world()
        except rospy.ServiceException as e:
                rospy.logerr('GeWorldProperties service call failed: {0}'.format(e))
                success = False
        for name in resp_world.model_names:
            if 'Object' in name:
                try:
                    resp_delete = self.model_remover(name)         
                except rospy.ServiceException as e:
                    rospy.logerr('Delete model service call failed: {0}'.format(e))
                    success = False
        rospy.loginfo('Models deleted')
        return success
        


    def callback(self, request):
        rospy.loginfo("Clearing objects from scene...")
        success = self.clear_scene()
        return ClearObjectsFromSceneResponse(success)


        
def start_clear_scene_server():
    rospy.init_node('elsa_clear_scene_server') 
    cleaner = SceneCleaner()
    rate = rospy.Rate(100)
    service = rospy.Service('benchmark/clear_scene', ClearObjectsFromScene, cleaner.callback)
    rospy.loginfo("Scene clearing service ready")
    rospy.spin()


if __name__ == '__main__':
    start_clear_scene_server()       
