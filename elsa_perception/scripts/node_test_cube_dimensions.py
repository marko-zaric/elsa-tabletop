import rospy
# from perception.pointcloud_bb import pointcloudBB
from sensor_msgs.point_cloud2 import PointCloud2
import sensor_msgs.point_cloud2 as pc2
import ctypes
import struct
import numpy as np
from matplotlib import colors
from elsa_perception_msgs.msg import PhysicalScene, ClusteredPointcloud
from perception.pointcloud_objects import PointCloudScene
import matplotlib.pyplot as plt
# sensor_msgs/PointCloud2

SCENE = None
DATA_CALLBACK = None
ENABLE_COLOR = True
SAVE_POINT_CLOUD = False
HSV_COLOR = True

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
    rospy.Subscriber("/downsample/output", PointCloud2, callback=callback) # 
    pub = rospy.Publisher('elsa_perception/scene_description', PhysicalScene, queue_size=1)
    pub_cluster = rospy.Publisher('elsa_perception/clustered_pointcloud', ClusteredPointcloud , queue_size=1)
    counter = 0
    while not rospy.is_shutdown():
        rospy.wait_for_message("/downsample/output", PointCloud2)
        gen = pc2.read_points(DATA_CALLBACK, skip_nans=True, field_names=("x", "y", "z", "rgb"))
        count_points = 0
        xyz = np.zeros((int(DATA_CALLBACK.height * DATA_CALLBACK.width), 3))
        color = np.zeros((DATA_CALLBACK.width, 3))
        for point in gen:
            xyz[count_points, :3] = [point[0], point[1], point[2]]
            if ENABLE_COLOR:
                rgb_float = point[3]
                # # cast float32 to int so that bitwise operations are possible
                s = struct.pack('>f' ,rgb_float)
                i = struct.unpack('>l',s)[0]


                # # you can get back the float value by the inverse operations
                pack = ctypes.c_uint32(i).value

                r = (pack & 0x00FF0000)>> 16
                g = (pack & 0x0000FF00)>> 8
                b = (pack & 0x000000FF)

                if HSV_COLOR:
                    color[count_points, :3] = colors.rgb_to_hsv([r / 255, g / 255, b / 255])
                else: 
                    color[count_points, :3] = [r, g, b]

            count_points += 1
        print(count_points)
        if SAVE_POINT_CLOUD:
            np.save("/home/marko/IIS/point_clouds/xyz_real_"+ str(counter) +".npy", xyz)
            if ENABLE_COLOR:
                if HSV_COLOR:
                    np.save("/home/marko/IIS/point_clouds/hsv.npy", color)
                else:
                    np.save("/home/marko/IIS/point_clouds/rgb.npy", color)
            print("saved")
        counter += 1

        PC = PointCloudScene()
        if not (xyz.shape == (0,3) or color.shape == (0,3)):
            PC.detect_objects(xyz, color)
            PC.create_bounding_boxes()
            for i, single_object in enumerate(PC.objects_in_scene):
                print("Relative size error obj "+ str(i) + ": ", (np.array(single_object.size())-0.052) / 0.052)
            fig = plt.figure()
            axis = fig.add_subplot(111, projection='3d')
            PC.plot_scene(axis)
            plt.show()
            PC.calculate_surface_features()
            SCENE = PC.create_physical_scene_msg()
            pub.publish(SCENE) 
            POINTCLOUD_CLUSTER = PC.create_clustered_pointcloud_msg()
            pub_cluster.publish(POINTCLOUD_CLUSTER)

    

if __name__ == '__main__':
    listener()
