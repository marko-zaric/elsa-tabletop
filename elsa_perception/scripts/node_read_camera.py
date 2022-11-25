import rospy
from perception.pointcloud_bb import pointcloudBB
from sensor_msgs.point_cloud2 import PointCloud2
import sensor_msgs.point_cloud2 as pc2
import ctypes
import struct
import numpy as np
from elsa_perception_msgs.msg import BoundingBox
from elsa_perception_msgs.msg import BB_Scene
# sensor_msgs/PointCloud2

arrayBBS = None
SCENE = None
DATA_CALLBACK = None

def ArrayToBBScene(bounding_boxes):
    scene = []
    for obj in bounding_boxes:
        bounding_box = BoundingBox()
        bounding_box.x = obj[0]
        bounding_box.y = obj[1]
        bounding_box.z = obj[2]
        bounding_box.phi = obj[3]
        bounding_box.dx = obj[4]
        bounding_box.dy = obj[5]
        bounding_box.dz = obj[6]
        scene.append(bounding_box)
    bb_scene = BB_Scene()
    bb_scene.boundingboxes = scene
    return bb_scene


def callback(data):
    global DATA_CALLBACK
    # rospy.loginfo("PointCloud Data: ")
    # rospy.loginfo(len(data.data))
    # rospy.loginfo("Row step: ")
    # rospy.loginfo(data.row_step)
    # rospy.loginfo("h: ")
    # rospy.loginfo(data.height)
    # rospy.loginfo("w: ")
    # rospy.loginfo(data.width)

    DATA_CALLBACK = data     



def listener():
    rospy.init_node("read_cam_data", anonymous=True)
    rospy.Subscriber("/downsample/output", PointCloud2, callback=callback) # /downsample/output
    pub = rospy.Publisher('/bounding_boxes', BB_Scene, queue_size=1)
    # s = rospy.Service('observe', Observe, handle_observe)
    while not rospy.is_shutdown():
        rospy.wait_for_message("/downsample/output", PointCloud2)
        gen = pc2.read_points(DATA_CALLBACK, skip_nans=True, field_names=("x", "y", "z", "rgb"))
        count_points = 0
        xyz = np.zeros((int(DATA_CALLBACK.height * DATA_CALLBACK.width), 3))
        rgb = np.zeros((int(DATA_CALLBACK.height * DATA_CALLBACK.width), 3))
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
        print(count_points)
        np.save("/home/marko/Desktop/IIS_Research/xyz_real.npy", xyz)
        np.save("/home/marko/Desktop/IIS_Research/rgb_real.npy", rgb)
        print("saved")

        SCENE = ArrayToBBScene(pointcloudBB(xyz))
        
        pub.publish(SCENE)
    

if __name__ == '__main__':
    listener()
