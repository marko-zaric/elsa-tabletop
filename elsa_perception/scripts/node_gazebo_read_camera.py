#!/usr/bin/env python3

import rospy
from perception.pointcloud_objects import PointCloudScene
from sensor_msgs.point_cloud2 import PointCloud2
import sensor_msgs.point_cloud2 as pc2
import ctypes
import struct
import numpy as np
from elsa_perception_msgs.msg import PhysicalScene, PhysicalSceneStamped, FullSceneStamped, SocialFeatures
from matplotlib import colors
import std_msgs.msg

SCENE = None
DATA_CALLBACK = None
ENABLE_COLOR = True
SAVE_POINT_CLOUD = False
HSV_COLOR = True


def callback(data):
    global DATA_CALLBACK
    DATA_CALLBACK = data     

def listener():
    rospy.init_node("read_cam_data", anonymous=True)
    rospy.Subscriber("/downsample/output", PointCloud2, callback=callback)
    pub = rospy.Publisher('/scene_description', PhysicalScene, queue_size=1)
    scene_stamped_pub = rospy.Publisher('/scene_description_stamped', PhysicalSceneStamped, queue_size=1)
    fullscene_stamped_pub = rospy.Publisher('/fullscene', FullSceneStamped, queue_size=1)
    seq_number = 0
    while not rospy.is_shutdown():
        rospy.wait_for_message("/downsample/output", PointCloud2)
        gen = pc2.read_points(DATA_CALLBACK, skip_nans=True, field_names=("x", "y", "z", "rgb"))
        count_points = 0

        xyz = np.zeros((DATA_CALLBACK.width, 3))
        color = np.zeros((DATA_CALLBACK.width, 3))
        for point in gen:
            if ENABLE_COLOR:
                rgb_float = point[3]
                # cast float32 to int so that bitwise operations are possible
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

            xyz[count_points, :3] = [point[0], point[1], point[2]]
            

            count_points += 1

        if SAVE_POINT_CLOUD:
            np.save("/home/marko/Desktop/IIS_Research/xyz.npy", xyz)
            if ENABLE_COLOR:
                if HSV_COLOR:
                    np.save("/home/marko/Desktop/IIS_Research/hsv.npy", color)
                else:
                    np.save("/home/marko/Desktop/IIS_Research/rgb.npy", color)


        PC = PointCloudScene()
        if not (xyz.shape == (0,3) or color.shape == (0,3)):
            PC.detect_objects(xyz, color)
            PC.create_bounding_boxes()
            PC.calculate_surface_features()
            SCENE = PC.create_physical_scene_msg()
            SCENE_stamped = PhysicalSceneStamped()
            SCENE_stamped.physical_scene = SCENE.physical_scene
            SCENE_stamped.header = std_msgs.msg.Header()
            SCENE_stamped.header.stamp = rospy.Time.now()
            SCENE_stamped.header.seq = seq_number
            
            FULLSCENE_stamped = FullSceneStamped()
            FULLSCENE_stamped.physical_scene = SCENE.physical_scene
            FULLSCENE_stamped.header = std_msgs.msg.Header()
            FULLSCENE_stamped.header.stamp = SCENE_stamped.header.stamp
            FULLSCENE_stamped.header.seq = SCENE_stamped.header.seq
            FULLSCENE_stamped.social_features.append(SocialFeatures())

            pub.publish(SCENE)
            scene_stamped_pub.publish(SCENE_stamped)
            fullscene_stamped_pub.publish(FULLSCENE_stamped)
            seq_number += 1
    

if __name__ == '__main__':
    listener()
