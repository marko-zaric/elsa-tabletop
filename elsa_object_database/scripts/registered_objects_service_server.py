import rospy 
from elsa_object_database.srv import RegisteredObjects, RegisteredObjectsResponse
from elsa_object_database.msg import ObjectRegistration
import csv 
import rospkg

def callback(request):
    list_registered_objects = []
    rospack = rospkg.RosPack()
    path_to_pkg = rospack.get_path('elsa_object_database')
    with open(path_to_pkg + '/ObjectDatabase.csv', newline='') as csvfile:
        csv_reader = csv.reader(csvfile, delimiter=',')
        for row in csv_reader:
            if 'Gazebo' in row[2]:
                obj = ObjectRegistration(object_name=row[0], 
                                         h_value=float(row[1]), 
                                         gazebo_color=row[2], 
                                         r_value=0.0, g_value=0.0, b_value=0.0, alpha_value=0.0)
            else:
                obj = ObjectRegistration(object_name=row[0], 
                                         h_value=float(row[1]), 
                                         gazebo_color='ambient color', 
                                         r_value=float(row[2]), 
                                         g_value=float(row[3]), 
                                         b_value=float(row[4]), 
                                         alpha_value=float(row[5]))
            list_registered_objects.append(obj)
    return RegisteredObjectsResponse(list_registered_objects)


def get_registered_objects():
    rospy.init_node('elsa_object_database')
    service = rospy.Service('get_registered_obj_service', RegisteredObjects, callback)
    rospy.spin()


if __name__ == '__main__':
    get_registered_objects()