import numpy as np
import matplotlib.pyplot as plt
from sklearn.cluster import DBSCAN
from mz_cv.bounding_box import *
from mz_cv.plane_removal import *


xyz = np.load("/home/marko/Desktop/IIS_Research/catkin_workspaces/panda_catkin_ws/src/panda_simulator/iis_panda_controls/scripts/xyz.npy")
rgb = np.load("/home/marko/Desktop/IIS_Research/catkin_workspaces/panda_catkin_ws/src/panda_simulator/iis_panda_controls/scripts/rgb.npy")

xyz[:,2] = 1 - xyz[:,2]
xyz[:,0] = xyz[:,0]
xyz[:,1] = - xyz[:,1]

# get frame
min_point, max_point = get_image_corners(xyz)


xyz, rgb = plane_removal(xyz, rgb)
ax = plt.axes(projection='3d')
ax.set_box_aspect((np.ptp(xyz[:,0]),np.ptp(xyz[:,1]), np.ptp(xyz[:,2])))

# DBSCAN 
dbscan = DBSCAN(eps=0.05, min_samples=10)
dbscan.fit(xyz)

labels_set = set(dbscan.labels_)

objects = []
obj_spartial_features = []
for i in range(len(labels_set)):
    objects.append([])

for i, xyz_ in zip(dbscan.labels_, xyz):
    objects[i].append(xyz_)


count = 0
for obj, name in zip(objects, ["Box1", "Box2", "Circle1", "Circle2", "Robot Foot"]):
    print(name)
    #spat_features = bounding_box_spartial_features(obj, ax)
    # print(spat_features)
xyz_plot = xyz
rgb_plot = rgb
ax.set_box_aspect((np.ptp(xyz_plot[:,0]),np.ptp(xyz_plot[:,1]), np.ptp(xyz_plot[:,2])))
ax.scatter(xyz_plot[:,0], xyz_plot[:,1], xyz_plot[:,2],c = dbscan.labels_ , s=0.01) # rgb_plot/255

# Camera Coordinate Axis
t = np.linspace(-0.2,0.2,100)
x_axis = np.outer(t, np.array([1,0])) 
y_axis = np.outer(t, np.array([0,1]))
ax.plot(x_axis[:,0], x_axis[:,1], np.zeros(len(x_axis[:,1])), 'red') #x
ax.plot(y_axis[:,0], y_axis[:,1], np.zeros(len(y_axis[:,1])), 'green') #y

# Corner Pixel to see full camera image
add_image_bounding_pixels(ax, min_point, max_point)
plt.show()

