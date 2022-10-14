import rospy
import numpy as np
import matplotlib.pyplot as plt
import sys
from iis_panda_controls.msg import BoundingBox
from perception.pointcloud_objects import PointCloudScene





def test():
    rospy.init_node("test_point_cloud", anonymous=True)

    # xyz = np.load("/home/marko/Desktop/IIS_Research/xyz_can.npy")
    xyz = np.load("/home/marko/Desktop/IIS_Research/catkin_workspaces/panda_catkin_ws/src/panda_simulator/iis_panda_controls/scripts/xyz.npy")
    # rgb = np.load("/home/marko/Desktop/IIS_Research/rgb_can.npy")

    PC = PointCloudScene(debug=True)

    PC.detect_objects(xyz)

    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    
    for i in range(len(PC.objects_in_scene)):
        PC.objects_in_scene[i].plot_point_cloud(ax)
    plt.show()

if __name__ == '__main__':
    test()
