import matplotlib.pyplot as plt
import numpy as np
from sklearn.cluster import DBSCAN
from mz_cv.bounding_box import *
from mz_cv.plane_removal import *

from mz_cv.rejection_sampling import uniform_convex_hull_sample
from scipy.spatial import ConvexHull


xyz = np.load("/home/marko/Desktop/IIS_Research/catkin_workspaces/panda_catkin_ws/src/panda_simulator/iis_panda_controls/scripts/xyz.npy")
rgb = np.load("/home/marko/Desktop/IIS_Research/catkin_workspaces/panda_catkin_ws/src/panda_simulator/iis_panda_controls/scripts/rgb.npy")

xyz[:,2] = 1.001 - xyz[:,2] # height of camera montage - z coordinates of points
xyz[:,0] = xyz[:,0]
xyz[:,1] = - xyz[:,1]

# --- Segmentation ---  
xyz, rgb = plane_removal(xyz, rgb, 10**-2)
dbscan = DBSCAN(eps=0.05, min_samples=10)
dbscan.fit(xyz)

labels_set = set(dbscan.labels_)

objects = []
obj_spartial_features = []
for i in range(len(labels_set)):
    objects.append([])

for i, xyz_ in zip(dbscan.labels_, xyz):
    objects[i].append(xyz_)

# --- Find maximal Simplex ---
max_simplex = 0
for obj, name in zip(objects, ["Box 1", "Box 2", "Circle 1", "Circle 2", "Robot Foot", "Brick 1", "Brick 2"]):
    print(name)
    points = np.array(objects[1])[:,0:2] 
    s_ps = uniform_convex_hull_sample(points, 1000)
    hull = ConvexHull(points)
    verts = hull.vertices
    max_dist = 0
    max_edge = None
    for simplex in hull.simplices:
        diff = points[simplex][0] - points[simplex][1]
        dist = np.linalg.norm(diff)
        if dist > max_dist:
            max_dist = dist
            max_simplex = simplex
            max_edge = points[simplex][0] - points[simplex][1]
    break


hull = ConvexHull(points)
verts = hull.vertices
max_dist = 0
max_edge = None


fig, (ax1, ax2) = plt.subplots(ncols=2, figsize=(10, 3))
A = 0
for ax in (ax1, ax2):
    ax.plot(s_ps[:, 0], s_ps[:, 1], '.', color='green', markersize=1)
    if ax == ax1:
        ax.set_title('Given points')
    else:
        ax.set_title('Convex hull')
        
        for simplex in hull.simplices:
            if all(max_simplex == simplex): # remove to plot all simplices
                ax.plot(points[simplex, 0], points[simplex, 1], 'c')
        ax.plot(points[verts, 0], points[verts, 1], 'o', mec='r', color='none', lw=1, markersize=10)
plt.show()