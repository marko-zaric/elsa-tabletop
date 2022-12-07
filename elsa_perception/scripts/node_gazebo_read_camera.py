#!/usr/bin/env python3

import rospy
from perception.pointcloud_objects import PointCloudScene
from sensor_msgs.point_cloud2 import PointCloud2
import sensor_msgs.point_cloud2 as pc2
import ctypes
import struct
import numpy as np
from elsa_perception_msgs.msg import BoundingBox
from elsa_perception_msgs.msg import BB_Scene
from matplotlib import colors

arrayBBS = None
SCENE = None
DATA_CALLBACK = None
ENABLE_COLOR = True
SAVE_POINT_CLOUD = False
HSV_COLOR = True

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
    DATA_CALLBACK = data     



def listener():
    rospy.init_node("read_cam_data", anonymous=True)
    rospy.Subscriber("/downsample/output", PointCloud2, callback=callback)
    pub = rospy.Publisher('/scene_description', BB_Scene, queue_size=1)
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
            # PC.calculate_surface_features()
            SCENE = ArrayToBBScene(PC.bounding_boxes)
            pub.publish(SCENE) 
    

if __name__ == '__main__':
    listener()
