import matplotlib.pyplot as plt
import numpy as np
from sklearn.cluster import DBSCAN
from sklearn.decomposition import PCA
from mz_cv.rejection_sampling import uniform_convex_hull_sample
from scipy.spatial import ConvexHull
from mz_cv.bounding_box import *
from mz_cv.plane_removal import *
import math as m

xyz = np.load("/home/marko/Desktop/IIS_Research/catkin_workspaces/panda_catkin_ws/src/panda_simulator/iis_panda_controls/scripts/xyz.npy")
rgb = np.load("/home/marko/Desktop/IIS_Research/catkin_workspaces/panda_catkin_ws/src/panda_simulator/iis_panda_controls/scripts/rgb.npy")

xyz[:,2] = 1 - xyz[:,2]
xyz[:,0] = xyz[:,0]
xyz[:,1] = - xyz[:,1]

xyz, rgb = plane_removal(xyz, rgb)
dbscan = DBSCAN(eps=0.05, min_samples=10)
dbscan.fit(xyz)

labels_set = set(dbscan.labels_)

objects = []
obj_spartial_features = []
for i in range(len(labels_set)):
    objects.append([])

for i, xyz_ in zip(dbscan.labels_, xyz):
    objects[i].append(xyz_)

# obj_num = 0

# points = np.array(objects[obj_num])[:,0:2] 
# pose, dimensions = convex_hull_bounding_box(objects[obj_num])
fig = plt.figure(figsize=(12,12))
ax = fig.add_subplot(111)

# ax.scatter(points[:,0],points[:,1], s=1)
# print(pose)
# print(dimensions)
# ax.scatter([pose[0]],[pose[1]])   
# plt.axis('equal')
# plt.show()
# exit()


max_simplex = 0
for obj, name in zip(objects, ["Box 1", "Box 2", "Circle 1", "Circle 2", "Robot Foot"]):
    print(name)
    points = np.array(obj)[:,0:2] 
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

    edge = max_edge/np.linalg.norm(max_edge)
    if edge[0] >= 0 and edge[1] >= 0:
        x = np.array([0, 1])
    elif edge[0] >= 0 and edge[1] < 0:
        x = np.array([1, 0])
    elif edge[0] < 0 and edge[1] < 0:
        x = np.array([0, -1])
    else:
        x = np.array([-1, 0])



    #y = np.array([1, 0])

    theta = m.acos(np.dot(x,edge)) 
    #theta2 = m.acos(np.dot(y,edge))
    print(edge)
    p1 = edge*0
    p2 = edge*(1)
    print(theta)
    #print(theta2/(2*np.pi)*360)
    ax.plot([p1[0], p2[0]], [p1[1], p2[1]], label=name)

ax.plot([0, 0], [0, 1], color="black")
ax.plot([0, 0], [0, -1], color="black")
ax.plot([0, 1], [0, 0], color="black")
ax.plot([0, -1], [0, 0], color="black")
ax.legend()
plt.show()

exit()
hull = ConvexHull(points)
verts = hull.vertices
max_dist = 0
max_edge = None


# fig, (ax1, ax2) = plt.subplots(ncols=2, figsize=(10, 3))
# A = 0
# for ax in (ax1, ax2):
#     ax.plot(s_ps[:, 0], s_ps[:, 1], '.', color='green', markersize=1)
#     if ax == ax1:
#         ax.set_title('Given points')
#     else:
#         ax.set_title('Convex hull')
        
#         for simplex in hull.simplices:
#             if all(max_simplex == simplex):
#                 ax.plot(points[simplex, 0], points[simplex, 1], 'c')
#         ax.plot(points[verts, 0], points[verts, 1], 'o', mec='r', color='none', lw=1, markersize=10)
# plt.show()


center = [0, 0] # x,y
dimensions = [0, 0] # x,y dimensions

min_x = np.min(s_ps[:,0])
min_y = np.min(s_ps[:,1])
max_x = np.max(s_ps[:,0])
max_y = np.max(s_ps[:,1])

# dimensions
dimensions[0] = max_x - min_x
dimensions[1] = max_y - min_y
#pose
center[0] = min_x + dimensions[0]/2
center[1] = min_y + dimensions[1]/2



ca = np.cov(s_ps,y = None,rowvar = 0,bias = 1)

v, vect = np.linalg.eig(ca)
#print(vect)
tvect = np.transpose(vect)




#use the inverse of the eigenvectors as a rotation matrix and
#rotate the points so they align with the x and y axes
ar = np.dot(s_ps,np.linalg.inv(tvect))

# get the minimum and maximum x and y 
mina = np.min(ar,axis=0)
maxa = np.max(ar,axis=0)
diff = (maxa - mina)*0.5

# the center is just half way between the min and max xy
center = mina + diff

#get the 4 corners by subtracting and adding half the bounding boxes height and width to the center
corners = np.array([center+[-diff[0],-diff[1]],center+[diff[0],-diff[1]],center+[diff[0],diff[1]],center+[-diff[0],diff[1]],center+[-diff[0],-diff[1]]])

#use the the eigenvectors as a rotation matrix and
#rotate the corners and the centerback
corners = np.dot(corners,tvect)
center = np.dot(center,tvect)


fig = plt.figure(figsize=(12,12))
ax = fig.add_subplot(111)

R = np.array([[np.cos(theta), -np.sin(theta)], [np.sin(theta), np.cos(theta)]])

points = np.dot(R, (points-center).T).T + center

ax.scatter(points[:,0],points[:,1], s=1)

ax.scatter([center[0]],[center[1]])    
#ax.plot(corners[:,0],corners[:,1],'-')

plt.axis('equal')
plt.show()