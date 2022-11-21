import rospy
import numpy as np
import matplotlib.pyplot as plt
from iis_panda_controls.msg import BoundingBox
from perception.pointcloud_objects import PointCloudScene

def add_image_bounding_pixels(ax, min_point, max_point):
    xyz = np.array([[min_point[0], min_point[1], max_point[2]]])
    xyz = np.append(xyz, [[min_point[0], max_point[1], max_point[2]]], axis = 0)
    xyz = np.append(xyz, [[max_point[0], min_point[1], max_point[2]]], axis = 0)
    xyz = np.append(xyz, [[max_point[0], max_point[1], max_point[2]]], axis = 0)

    ax.scatter(xyz[:,0], xyz[:,1], xyz[:,2], s=0.0001)

def test():
    rospy.init_node("test_point_cloud", anonymous=True)

    xyz = np.load("/home/marko/Desktop/IIS/xyz.npy")
    rgb = np.load("/home/marko/Desktop/IIS/rgb.npy")

    PC = PointCloudScene(debug=False)

    PC.detect_objects(xyz)
    PC.create_bounding_boxes()

    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    add_image_bounding_pixels(ax, np.array([-0.5, -0.5, 0]), np.array([0.5, 0.5, 0.5]))
    for i in range(len(PC.objects_in_scene)):
        PC.objects_in_scene[i].plot_point_cloud(ax=ax, marker_size=1)
        PC.objects_in_scene[i].plot_bounding_box(ax=ax)
    plt.show()

if __name__ == '__main__':
    test()