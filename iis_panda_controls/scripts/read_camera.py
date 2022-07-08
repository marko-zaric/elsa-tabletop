import rospy
import sys
sys.path.append("/home/marko/Desktop/IIS_Research/catkin_workspaces/panda_catkin_ws/src/panda_simulator/iis_panda_controls/scripts")
from pointcloud_bb import pointcloudBB
from sensor_msgs.point_cloud2 import PointCloud2
import sensor_msgs.point_cloud2 as pc2
import ctypes
import struct
import numpy as np
import time
# sensor_msgs/PointCloud2
def callback(data):
    rospy.loginfo("PointCloud Data: ")
    rospy.loginfo(len(data.data))
    rospy.loginfo("Row step: ")
    rospy.loginfo(data.row_step)
    rospy.loginfo("h: ")
    rospy.loginfo(data.height)
    rospy.loginfo("w: ")
    rospy.loginfo(data.width)
    gen = pc2.read_points(data, skip_nans=True, field_names=("x", "y", "z", "rgb"))
    count_points = 0
    xyz = np.zeros((data.width, 3))
    rgb = np.zeros((data.width, 3)) # 921600
    st = time.time()
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


    et = time.time()
    elapsed_time = et - st
    print(count_points)
    print('Execution time xyz rgb extract:', elapsed_time, 'seconds')
    st = time.time()
    pointcloudBB(xyz, rgb)
    et = time.time()
    elapsed_time = et - st
    print('Execution time boundingbox creation:', elapsed_time, 'seconds')

    # np.save("/home/marko/Desktop/IIS_Research/catkin_workspaces/panda_catkin_ws/src/panda_simulator/iis_panda_controls/scripts/xyz", xyz)
    # np.save("/home/marko/Desktop/IIS_Research/catkin_workspaces/panda_catkin_ws/src/panda_simulator/iis_panda_controls/scripts/rgb", rgb)
    rospy.signal_shutdown("Goal reached")
    
        

def listener():
    rospy.init_node("read_cam_data", anonymous=True)
    rospy.Subscriber("/downsample/output", PointCloud2, callback=callback) # /camera/depth/color/points
    rospy.spin()

if __name__ == '__main__':
    listener()
