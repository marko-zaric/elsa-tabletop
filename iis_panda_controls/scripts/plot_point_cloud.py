import numpy as np
import matplotlib.pyplot as plt
from matplotlib import collections  as mc
import itertools
from sklearn.cluster import DBSCAN
from sklearn.decomposition import PCA

xyz = np.load("/home/marko/Desktop/IIS_Research/catkin_workspaces/panda_catkin_ws/src/panda_simulator/iis_panda_controls/scripts/xyz.npy")
rgb = np.load("/home/marko/Desktop/IIS_Research/catkin_workspaces/panda_catkin_ws/src/panda_simulator/iis_panda_controls/scripts/rgb.npy")

xyz[:,2] = 1 - xyz[:,2]

mean=np.mean(xyz, axis=0)

def rgb_dist(rgb_vec):
    return np.sqrt((rgb_vec[0])**2 + (rgb_vec[1])**2 + (rgb_vec[2])**2)

def primitive_bounding_box(object):
    if not isinstance(object, np.ndarray):
        object = np.array(object)

    min_x = np.min(object[:,0])
    min_y = np.min(object[:,1])
    min_z = np.min(object[:,2])
    max_x = np.max(object[:,0])
    max_y = np.max(object[:,1])
    max_z = np.max(object[:,2])
    x = [min_x, max_x]
    y = [min_y, max_y]
    z = [min_z, max_z]
    corner_points = np.array(list(itertools.product(x, y, z)))
    
    return corner_points

def plt_bounding_box(ax, corner_points):
    ax.scatter(corner_points[:,0], corner_points[:,1], corner_points[:,2], marker='^')

def bounding_box_spartial_features(object):
    pose = [0, 0, 0, 0] # x,y,z of center of bounding box; theta
    dimensions = [0, 0, 0] # x,y,z dimensions

    if not isinstance(object, np.ndarray):
        object = np.array(object)

    min_x = np.min(object[:,0])
    min_y = np.min(object[:,1])
    min_z = np.min(object[:,2])
    max_x = np.max(object[:,0])
    max_y = np.max(object[:,1])
    max_z = np.max(object[:,2])
    
    # dimensions
    dimensions[0] = max_x - min_x
    dimensions[1] = max_y - min_y
    dimensions[2] = max_z - min_z

    #pose
    pose[0] = min_x + dimensions[0]/2
    pose[1] = min_y + dimensions[1]/2
    pose[2] = min_z + dimensions[2]/2
    return (pose, dimensions)

def add_image_bounding_pixels(ax, min_point, max_point):
    xyz = np.array([[min_point[0], min_point[1], max_point[2]]])
    xyz = np.append(xyz, [[min_point[0], max_point[1], max_point[2]]], axis = 0)
    xyz = np.append(xyz, [[max_point[0], min_point[1], max_point[2]]], axis = 0)
    xyz = np.append(xyz, [[max_point[0], max_point[1], max_point[2]]], axis = 0)

    ax.scatter(xyz[:,0], xyz[:,1], xyz[:,2], s=0.0001)


# get frame
min_x = np.min(xyz[:,0])
min_y = np.min(xyz[:,1])
min_z = np.min(xyz[:,2])
max_x = np.max(xyz[:,0])
max_y = np.max(xyz[:,1])
max_z = 0.5 #np.max(xyz[:,2])
min_point = [min_x, min_y, min_z]
max_point = [max_x, min_y, max_z] 


# Background detection
table_z = min(xyz[:,2][np.nonzero(xyz[:,2])])
rgb_distances = np.sqrt((rgb[:,0])**2 + (rgb[:,1])**2 + (rgb[:,2])**2)
bins = np.histogram(rgb_distances, bins=1000)
background = bins[1][np.argmax(bins[0])]

xyz_planeless = []
rgb_planeless = []
# plane removal
for i, (xyz_, rgb_) in enumerate(zip(xyz, rgb)):
    #if abs(background - rgb_dist(rgb_)) < 2:
    if abs(xyz_[2] - mean[2]) > 10**-2:
        xyz_planeless.append(xyz_)
        rgb_planeless.append(rgb_)


xyz_planeless = np.array(xyz_planeless)
rgb_planeless = np.array(rgb_planeless)



# DBSCAN 
# dbscan = DBSCAN(eps=0.05, min_samples=10)
# dbscan.fit(xyz_planeless)

# labels_set = set(dbscan.labels_)

# objects = []
# obj_spartial_features = []
# for i in range(len(labels_set)):
#     objects.append([])
# for i, xyz_ in zip(dbscan.labels_, xyz_planeless):
#     objects[i].append(xyz_)

ax = plt.axes(projection='3d')
# count = 0
# for obj in objects:
#     objects[count] = np.array(obj)
#     count += 1
#     c_points = primitive_bounding_box(obj)
#     plt_bounding_box(ax, c_points)
#     spat_features = bounding_box_spartial_features(obj)
#     print(spat_features)
#     ax.scatter(spat_features[0][0], spat_features[0][1], spat_features[0][2])

# plot pca axis: 
# pca = PCA(n_components=2)
# X2D = pca.fit_transform(np.array(objects[0])[:,0:2])
# print(pca.explained_variance_ratio_)
# print(pca.components_)
# t = np.linspace(-0.2,0.2,100)
# ax_line = np.outer(t, pca.components_[0]) + np.array([-0.30003, 0.15004808])
# print(ax_line.shape)
# ax_line2 = np.outer(t, pca.components_[1]) + np.array([-0.30003, 0.15004808])

xyz_plot = xyz_planeless
rgb_plot = rgb_planeless
ax.set_box_aspect((np.ptp(xyz_plot[:,0]),np.ptp(xyz_plot[:,1]), np.ptp(xyz_plot[:,2])))
ax.scatter(xyz_plot[:,0], xyz_plot[:,1], xyz_plot[:,2],c =  rgb_plot/255, s=0.01) #dbscan.labels_
#ax.scatter(objects[0][:,0], objects[0][:,1], objects[0][:,2], s=0.01)

# PCA Axis
# ax.plot(ax_line[:,0], ax_line[:,1], np.ones(len(ax_line[:,1]))*0.06812, 'red')
# ax.plot(ax_line2[:,0], ax_line2[:,1], np.ones(len(ax_line2[:,1]))*0.06812, 'red')

# Coordinates
# ax.scatter(0.1,0,0, c='red')
# ax.scatter(0,0.1,0, c='green')

# Corner Pixel to see full camera image
add_image_bounding_pixels(ax, min_point, max_point)
plt.show()

