import rospy
import numpy as np
import matplotlib.pyplot as plt
from perception.pointcloud_objects import PointCloudScene
import time


def add_image_bounding_pixels(ax, min_point, max_point):
    xyz = np.array([[min_point[0], min_point[1], max_point[2]]])
    xyz = np.append(xyz, [[min_point[0], max_point[1], max_point[2]]], axis = 0)
    xyz = np.append(xyz, [[max_point[0], min_point[1], max_point[2]]], axis = 0)
    xyz = np.append(xyz, [[max_point[0], max_point[1], max_point[2]]], axis = 0)

    ax.scatter(xyz[:,0], xyz[:,1], xyz[:,2], s=0.0001)

def test():
    rospy.init_node("test_point_cloud", anonymous=True)

    xyz = np.load("/home/marko/Desktop/IIS_Research/xyz.npy")
    # rgb = np.load("/home/marko/Desktop/IIS_Research/rgb.npy")

    PC = PointCloudScene(debug=False)

    PC.detect_objects(xyz)
    print(PC.calculate_surface_features())

    # fig = plt.figure()
    # ax = fig.add_subplot(111, projection='3d')

    # for i in range(len(PC.objects_in_scene)):
    #     PC.objects_in_scene[i].plot_point_cloud(ax=ax, marker_size=1)
    #     PC.objects_in_scene[i].plot_bounding_box(ax=ax)
    # plt.show()

if __name__ == '__main__':
    test()



exit()

def is_outlier(points, thresh=3.5):
    """
    Returns a boolean array with True if points are outliers and False 
    otherwise.

    Parameters:
    -----------
        points : An numobservations by numdimensions array of observations
        thresh : The modified z-score to use as a threshold. Observations with
            a modified z-score (based on the median absolute deviation) greater
            than this value will be classified as outliers.

    Returns:
    --------
        mask : A numobservations-length boolean array.

    References:
    ----------
        Boris Iglewicz and David Hoaglin (1993), "Volume 16: How to Detect and
        Handle Outliers", The ASQC Basic References in Quality Control:
        Statistical Techniques, Edward F. Mykytka, Ph.D., Editor. 
    """
    if len(points.shape) == 1:
        points = points[:,None]
    median = np.median(points, axis=0)
    diff = np.sum((points - median)**2, axis=-1)
    diff = np.sqrt(diff)
    med_abs_deviation = np.median(diff)

    modified_z_score = 0.6745 * diff / med_abs_deviation

    return modified_z_score > thresh

pc = np.array(cola.priciple_curvatures).T[0]
pc2 = np.array(cola.priciple_curvatures).T[1]
filtered = pc[~is_outlier(pc)]
filtered2 = pc2[~is_outlier(pc2)]

# pc_sort = np.sort(pc)
# vals = pc_sort[:20]
# index_min_curve = []
# out_index = 0
# for val in vals:
#     index_min_curve.append(np.where(pc == val)[0][0])
#     out_index = index_min_curve[0]
#     break


# print(index_min_curve)

# outlier = cola.xyz_points[index_min_curve[0]]

# index_min_curve = []
# for i, point in enumerate(cola.xyz_points):
#     if abs(outlier[0] - point[0]) < 10**(-3) and abs(outlier[1] - point[1]) < 10**(-3):
#         index_min_curve.append(i)


# plt.scatter(pc[index_min_curve], pc2[index_min_curve], color="black")
# plt.scatter(pc[out_index], pc2[out_index], color="red")
# plt.show()
# exit()


# print(np.array(cola.priciple_curvatures)[index_min_curve])

# hist, bin_eges = np.histogram(pc, bins=20)

# print(hist)
# print(bin_eges)
plt.hist(filtered, bins = 20)
plt.show()

plt.hist(filtered2, bins = 20)
plt.show()


# hist_1 = np.histogram(pc.T[1], bins=20)
# print(hist_1)
# print(np.max(pc.T[1]))
# print(np.min(pc.T[1]))





#cola.plot_surface_normals()#special_list=index_min_curve)
plt.show()





exit()

rgb_plot = np.array(colors[0]) / 255
ax.set_box_aspect((np.ptp(object[:,0]),np.ptp(object[:,1]), np.ptp(object[:,2])))
ax.scatter(object[:,0], object[:,1], object[:,2],c = rgb_plot , s=0.01) 



# Corner Pixel to see full camera image
add_image_bounding_pixels(ax, min_point, max_point)
plt.show()