import rospy
from sensor_msgs.point_cloud2 import PointCloud2
import sensor_msgs.point_cloud2 as pc2
import ctypes
import struct
import numpy as np
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
    xyz = np.zeros((921600, 3))
    rgb = np.zeros((921600, 3)) # 307200
    for point in gen:
        print(point)
        # rgb_float = point[3]
        # # cast float32 to int so that bitwise operations are possible
        # s = struct.pack('>f' ,rgb_float)
        # i = struct.unpack('>l',s)[0]
        # # you can get back the float value by the inverse operations
        # pack = ctypes.c_uint32(i).value
        # r = (pack & 0x00FF0000)>> 16
        # g = (pack & 0x0000FF00)>> 8
        # b = (pack & 0x000000FF)
        # print(f'r={r},g={g}, b={b}')
        # print(rgb_float)
        # xyz[count_points, 0] = point[0]
        # xyz[count_points, 1] = point[1]
        # xyz[count_points, 2] = point[2]
        # rgb[count_points, 0] = r
        # rgb[count_points, 1] = g
        # rgb[count_points, 2] = b
        # count_points += 1
    print(count_points)
    
    #np.save("/home/marko/Desktop/IIS_Research/catkin_workspaces/panda_catkin_ws/src/panda_simulator/iis_panda_controls/scripts/xyz", xyz)
    #np.save("/home/marko/Desktop/IIS_Research/catkin_workspaces/panda_catkin_ws/src/panda_simulator/iis_panda_controls/scripts/rgb", rgb)
    rospy.signal_shutdown("Goal reached")
    
        

def listener():
    rospy.init_node("read_cam_data", anonymous=True)
    rospy.Subscriber("/camera/depth/color/points", PointCloud2, callback=callback) # /camera/depth/points
    rospy.spin()

if __name__ == '__main__':
    listener()
