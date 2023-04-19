import matplotlib.pyplot as plt
import numpy as np
from sklearn.cluster import DBSCAN
from pointcloud_objects import PointCloudObject, PointCloudScene
import time
from bounding_box import *
from elsa_perception.scripts.perception.plot_utils import *


def pointcloudBB(xyz, rgb=None, ax = None):
    benchmark = []
    benchmark.append(time.time())
    if ax == None:
        xyz[:,2] = 1.001 - xyz[:,2]  #
        xyz[:,0] = xyz[:,0]
        xyz[:,1] = - xyz[:,1]
    benchmark.append(time.time())
    
    # --- Segmentation ---  
    #xyz, rgb = plane_removal(xyz, rgb, 10**-2)
    #benchmark.append(time.time())
    print(type(xyz.dtype))
    print(xyz.shape)
    print(np.max(xyz[:,2]))
    print(np.min(xyz[:,2]))
    dbscan = DBSCAN(eps=0.02, min_samples=6)
    dbscan.fit(xyz)

    labels_set = set(dbscan.labels_)
    benchmark.append(time.time())
    
    benchmark.append(time.time())
    objects = []
    for i in range(len(labels_set)):
        objects.append([])

    for i, xyz_ in zip(dbscan.labels_, xyz):
        objects[i].append(xyz_)
    benchmark.append(time.time())
    
    object_bounding_boxes = []
    # --- Bounding Boxes ---
    for obj_num in range(len(objects)):
        #print(labels[obj_num])
        pose_cam, pose_fixed, dimensions = convex_hull_bounding_box(objects[obj_num])

        #print(list(pose_fixed) + dimensions)
        object_bounding_boxes.append(list(pose_fixed) + dimensions)
        if ax != None:
            plt_bounding_box(ax, pose_cam, dimensions, label=obj_num)
    benchmark.append(time.time())

    for i in range(len(benchmark)-1):
        print("Milestone ", i , " time: ", benchmark[i+1]-benchmark[i])

    if ax != None:
        ax.scatter(xyz[:,0], xyz[:,1], xyz[:,2],c = dbscan.labels_, s=0.01)
        plt.legend()
        plt.show()  
    return object_bounding_boxes

 
if __name__ == '__main__':
    xyz = np.load("/home/marko/Desktop/IIS_Research/catkin_workspaces/panda_catkin_ws/src/panda_simulator/iis_panda_controls/scripts/xyz.npy")
    # rgb = np.load("/home/marko/Desktop/IIS_Research/catkin_workspaces/panda_catkin_ws/src/panda_simulator/iis_panda_controls/scripts/rgb.npy")
    
      # --- Plotting ---
    ax = plt.axes(projection='3d')
    ax.set_box_aspect((np.ptp(xyz[:,1]),np.ptp(xyz[:,1]), np.ptp(xyz[:,2])))
    # fig = plt.figure(figsize=(12,12))
    # ax = fig.add_subplot(111)
    
    PC = PointCloudScene(debug=True)
    PC.create_bounding_boxes(xyz)
    PC.plot_scene(ax)

    #pointcloudBB(xyz, ax=ax)






