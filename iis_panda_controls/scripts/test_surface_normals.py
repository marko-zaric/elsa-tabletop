import numpy as np
import matplotlib.pyplot as plt
from sklearn.cluster import DBSCAN
import sys
sys.path.append("/home/marko/Desktop/IIS_Research/catkin_workspaces/panda_catkin_ws/src/panda_simulator/iis_panda_controls/scripts/")
from mz_cv.bounding_box import *
from mz_cv.plane_removal import *
from mz_cv.pointcloud_objects import PointCloudObject
from sklearn.cluster import DBSCAN
#from elsa_perception.pointcloud_objects import PointCloudObject
import time

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


xyz = np.load("xyz_low_sample.npy")
rgb = np.load("rgb_low_sample.npy")

xyz[:,2] = 1.001 - xyz[:,2] # 0.851
xyz[:,0] = xyz[:,0]
xyz[:,1] = - xyz[:,1]

# get frame
min_point, max_point = get_image_corners(xyz)


# xyz_c = []
# rgb_c = []
# for _xyz, _rgb in zip(xyz, rgb):
#     if _xyz[2] > 0.1 or _xyz[2] < 0.0:
#         continue
#     else:
#         xyz_c.append(_xyz)
#         rgb_c.append(_rgb)
# xyz = np.array(xyz_c)
# rgb = np.array(rgb_c)

# xyz, rgb = plane_removal(xyz, rgb, 5*10**-2)


dbscan = DBSCAN(eps=0.03, min_samples=6)
dbscan.fit(xyz)
labels_set = set(dbscan.labels_)
objects = []
colors = []
for i in range(len(labels_set)):
    objects.append([])
    colors.append([])
for i, xyz_,rgb_ in zip(dbscan.labels_, xyz, rgb):
    objects[i].append(xyz_)
    colors[i].append(rgb_)

# ax = plt.axes(projection='3d')
# ax.scatter(xyz[:,0], xyz[:,1], xyz[:,2],c = dbscan.labels_ , s=1) 

# # Corner Pixel to see full camera image
# add_image_bounding_pixels(ax, min_point, max_point)
# plt.show()
# exit()
object = np.array(objects[5])

# rgb_plot = np.array(colors[0]) / 255
# #object = xyz
# ax.set_box_aspect((np.ptp(object[:,0]),np.ptp(object[:,1]), np.ptp(object[:,2])))
# ax.scatter(object[:,0], object[:,1], object[:,2],c = rgb_plot , s=1) 
# add_image_bounding_pixels(ax, min_point, max_point)
# plt.show()
# exit()
benchmark = []
benchmark.append(time.time())
cola = PointCloudObject(object)
benchmark.append(time.time())
cola.compute_bounding_box()
benchmark.append(time.time())
cola.compute_surface_normals()
benchmark.append(time.time())
cola.plot_surface_normals()
plt.show()

# SCENE = []
# for obj in objects:
#     SCENE.append(PointCloudObject(obj))

# benchmark.append(time.time())
# for obj in SCENE:
#     obj.compute_bounding_box()
# benchmark.append(time.time())
# for obj in SCENE:
#     obj.compute_surface_normals()
# benchmark.append(time.time())



# for i in range(len(benchmark)-1):
#     print("Milestone ", i , " time: ", benchmark[i+1]-benchmark[i])


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