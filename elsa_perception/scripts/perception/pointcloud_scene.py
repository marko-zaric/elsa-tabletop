import numpy as np
import matplotlib.pyplot as plt
import time
from sklearn.neighbors import NearestNeighbors
from sklearn.cluster import DBSCAN
from elsa_object_database.srv import RegisteredObjects
from elsa_perception_msgs.msg import PhysicalScene, ClusteredPointcloud
import rospy
import sensor_msgs.msg as sensor_msgs
import std_msgs.msg as std_msgs
from matplotlib import colors
from perception.real_world_preprocessing import remove_plane
from perception.pointcloud_objects import PointCloudObject
from scipy.spatial import distance_matrix


'''
This class describes a scene and its objects according to the paper Learning Social Affordances and Using Them for Planning 
(https://escholarship.org/content/qt9cj412wg/qt9cj412wg.pdf) by Uyanik et. al (2013)
The properties are divided into surface and spatial features. 
- Spatal features are bounding box pose (x, y, z, theta) and bounding box dimensions (x, y, z)

- Surface features are descibed as surface normals (azimuth and zenith), principal curvatures (min and max) and
    shape indices. They are represented as a 20-bin histogram in addition to the min, max, mean, standard deviation 
    and variance information.
'''

MEAN_PANDA_FOOT = np.array([ 0.01781263, -0.46853645,  0.04416075])

COLOR_SEGMENTATION = True

class PointCloudScene:
    def __init__(self, debug=False, register_objects=True):
        self.objects_in_scene = []
        self.xyz = None
        self.hsv = None
        self.dbscan_labels = None
        self.DEBUG = debug
        self.bounding_boxes = []

        self.registered_objs = []
        self.registered_objs_values = np.empty((2,))
        if debug is False:
            self.color_eps = 0.075
            self.register_objects = False
        else:
            self.color_eps = 0.05 #0.2
            # registered objects Sim only
            self.register_objects = register_objects
            if self.register_objects: self.registered_obj_client()    

    
    def detect_objects(self, xyz, hsv=None):
        self.hsv = hsv

        print("Detecting Objects...")
        benchmark = []
        benchmark.append(time.time())
        self.xyz = xyz
        self.xyz[:,0] = xyz[:,0]
        self.xyz[:,1] = - xyz[:,1]
        
        if self.DEBUG == False: 
            self.xyz[:,2] = 0.946 - xyz[:,2]
            self.xyz, self.hsv = remove_plane(self.xyz, hsv, 0.01)
        else:
            self.xyz[:,2] = 1.001 - xyz[:,2]

        benchmark.append(time.time())

        dbscan = DBSCAN(eps=0.022, min_samples=6) 
        dbscan.fit(self.xyz)
        self.dbscan_labels = dbscan.labels_

        labels_set = set(dbscan.labels_)
        benchmark.append(time.time())

        benchmark.append(time.time())
        objects = []
        objects_color = None
        for i in range(len(labels_set)):
            objects.append([])

        for i, xyz_ in zip(dbscan.labels_, self.xyz):
            objects[i].append(xyz_)
        benchmark.append(time.time())

        if hsv is not None:
            objects_color = []
            for i in range(len(labels_set)):
                objects_color.append([])

            for i, rgb_ in zip(dbscan.labels_, self.hsv):
                objects_color[i].append(rgb_)

        # for i in range(len(benchmark)-1):
        #     print("Milestone ", i , " time: ", benchmark[i+1]-benchmark[i])

        for i, obj in enumerate(objects):
            # Remove Panda foot from object list (sim mean corrdinates at the moment)
            if np.linalg.norm(np.array(obj).mean(axis=0) - MEAN_PANDA_FOOT) > 0.01:
                if len(obj) > 10:
                    if hsv is None:
                        self.objects_in_scene.append(PointCloudObject(obj))
                    else:
                        self.objects_in_scene.append(PointCloudObject(obj, objects_color[i]))
        
        '''
        If color segmentation is on and objects have color values:
        Perform h-space color clustering to detect objects which are pressed up against each other.
        Otherwise the objects in scene are the spartially clustered ones disregarding color.
        '''
        if hsv is not None and COLOR_SEGMENTATION == True:
            new_objects_in_scene = []      
            for obj in self.objects_in_scene:
                # Parametrize h value to circle in for cyclical clustering in 2D h-space 
                h_value = np.array(obj.hsv)[:,0] * 2*np.pi
                h_x = np.sin(h_value)
                h_y = np.cos(h_value)
                h_circle = np.concatenate((np.atleast_2d(h_x).T, np.atleast_2d(h_y).T), axis=1)
                
                # cluster detected object by color
                cluster = DBSCAN(eps=self.color_eps, min_samples=8) 
                cluster.fit(h_circle)


                # If dbscan color cluster results in more than one color determine objects by color clustering
                list_of_labels = list(set(cluster.labels_))
                colors_detected = len(set(cluster.labels_))
                
                if colors_detected > 1:
                    # Split the xyz values into objects according to color cluster
                    new_objs = [] 
                    new_objs_colors = []
                    for i in range(len(set(cluster.labels_))):
                        new_objs.append([])
                        new_objs_colors.append([])
                    for i, xyz_, hsv_ in zip(cluster.labels_, obj.xyz_points, obj.hsv):
                        new_objs[list_of_labels.index(i)].append([xyz_[0], xyz_[1], xyz_[2]])
                        new_objs_colors[list_of_labels.index(i)].append([hsv_[0], hsv_[1], hsv_[2]])

                    # find all small objects in color cluster (less than 30 points)
                    new_objs_copy = new_objs.copy()
                    list_of_small_objs = []
                    for indx in range(len(set(cluster.labels_))):
                        if len(new_objs_copy[indx]) < 30:
                            list_of_small_objs.append(indx)

                    # remove all small objects
                    list_of_small_objs.reverse()
                    for i in list_of_small_objs:
                        new_objs.pop(i)
                        new_objs_colors.pop(i)
                    
                    # remove statistical outlier points in xyz-space from color clustered object 
                    for indx in range(len(new_objs)):
                        arr_xyz = np.array(new_objs[indx])
                        arr_col = np.array(new_objs_colors[indx])
                        model = DBSCAN(eps = get_eps(new_objs[indx]), min_samples = 10).fit(arr_xyz)
                        cols_ = model.labels_ +100
                        biggest_cluster_label = np.bincount(cols_).argmax()
                        arr_xyz = arr_xyz[~(cols_ != biggest_cluster_label)]
                        arr_col = arr_col[~(cols_ != biggest_cluster_label)]
                        # fig = plt.figure()
                        # axis = fig.add_subplot(111, projection='3d')
                        # add_image_bounding_pixels(axis, np.array([-0.3, -0.3, 0]), np.array([0.3, 0.3, 0.3]))
                        
                        # scatter = axis.scatter(arr[:,0],arr[:,1],arr[:,2], c=cols_)
                        # axis.legend(*scatter.legend_elements())
                        # plt.show()
                        new_objects_in_scene.append(PointCloudObject(arr_xyz.tolist(), arr_col.tolist()))
                else:
                    new_objects_in_scene.append(obj)
            self.objects_in_scene = new_objects_in_scene

        # self.create_bounding_boxes()
        # for i, obj in enumerate(self.objects_in_scene):
        #     fig = plt.figure()
        #     axis = fig.add_subplot(111, projection='3d')
        #     h_val = np.array(obj.hsv)[:,0]
        #     new_stack = (np.vstack((h_val,np.ones_like(h_val),np.ones_like(h_val)))).T
        #     add_image_bounding_pixels(axis, np.array([-0.3, -0.3, 0]), np.array([0.3, 0.3, 0.3]))
        #     axis.scatter(obj.xyz_points[:,0], 
        #                  obj.xyz_points[:,1], 
        #                  obj.xyz_points[:,2], 
        #                  s=20, 
        #                  c = colors.hsv_to_rgb(new_stack)
        #                  )
        #     obj.plot_bounding_box(axis)
        #     plt.show()
        # exit()
            

    def create_bounding_boxes(self):
        print("Computing Bounding boxes ...")
        benchmark = []

        object_bounding_boxes = []
        benchmark.append(time.time())
        for i in range(len(self.objects_in_scene)):
            self.objects_in_scene[i].compute_bounding_box()
            # print(self.objects_in_scene[i].pose() + self.objects_in_scene[i].size())
            object_bounding_boxes.append(list(self.objects_in_scene[i].pose()) + self.objects_in_scene[i].size())
        benchmark.append(time.time())

        # for i in range(len(benchmark)-1):
        #     print("Milestone ", i , " time: ", benchmark[i+1]-benchmark[i])

        self.bounding_boxes = object_bounding_boxes
        return object_bounding_boxes
    
    def calculate_surface_features(self):
        print("Computing Surface features ...")
        # --- Surface Features ---
        for i in range(len(self.objects_in_scene)):
            self.objects_in_scene[i].compute_surface_normals()

    def create_physical_scene_msg(self):
        list_physical_features = []

        # Unique object constraint
        mean_h_spaces = []
        var_h_spaces = []
        for obj in self.objects_in_scene:
            h_values = np.array(obj.hsv)[:,0]
            xs = np.cos(np.pi*2*h_values)
            ys = np.sin(np.pi*2*h_values)
            h_space = np.vstack((xs,ys)).T
            mean_h_space = np.mean(h_space, axis=0)
            var_h_space = np.var(h_space, axis=0)[0] + np.var(h_space, axis=0)[1]
            mean_h_spaces.append(mean_h_space)
            var_h_spaces.append(var_h_space)
        
        mean_h_spaces = np.array(mean_h_spaces)
        dist_mat = distance_matrix(self.registered_objs_values,mean_h_spaces)

        # axis 1 registered objects perspective
        # axis 0 observed objects perspective
        min_dists = np.min(dist_mat, axis=0)
        min_dists_idx = np.argmin(dist_mat, axis=0)
        reg_obj_ids, counts = np.unique(min_dists_idx, return_counts=True)
        count_dict = dict(zip(reg_obj_ids, counts))
        
        labels = {}
        for i in range(len(self.registered_objs)):
            if i in count_dict:
                if count_dict[i] == 1:
                    labels[np.where(min_dists_idx == i)[0][0]] = self.registered_objs[i]
                if count_dict[i] > 1:
                    conflicting_objects = np.where(min_dists_idx == i)
                    min_var = 1000
                    min_var_idx = -1
                    for indx in conflicting_objects:
                        if min_var > var_h_spaces[indx]:
                            min_var = var_h_spaces[indx]
                            min_var_idx = indx
                    labels[min_var_idx] = self.registered_objs[i]
                    for i in conflicting_objects:
                        if i == min_var_idx:
                            continue
                        labels[i] = "High variance dropped out!"
            else:
                labels[np.where(min_dists_idx == i)[0][0]] = "Unregistered Object!"              

        for i, obj in enumerate(self.objects_in_scene):
            list_physical_features.append(obj.create_physical_features_msg(labels[i], self.register_objects))

        physical_scene = PhysicalScene(physical_scene=list_physical_features, number_of_objects=len(self.objects_in_scene))

        return physical_scene

    def registered_obj_client(self):
        rospy.wait_for_service("get_registered_obj_service")
        self.registered_objs = []
        self.registered_objs_values = np.empty((2,))
        try:
            get_registered_objs = rospy.ServiceProxy("get_registered_obj_service", RegisteredObjects)
            response = get_registered_objs()
            for i, obj in enumerate(response.registered_objects):
                self.registered_objs.append(obj.object_name)
                x = np.cos(obj.h_value*2*np.pi)
                y = np.sin(obj.h_value*2*np.pi)
                if i == 0:
                    self.registered_objs_values = np.array([[x,y]])
                else:
                    self.registered_objs_values = np.vstack((self.registered_objs_values, np.array([[x,y]])))
        except rospy.ServiceException as e:
            print("Registered Objects Service failed %s", e)


    def create_clustered_pointcloud_msg(self):
        points_with_label = np.empty((4,), dtype=np.float32)
        for i, obj in enumerate(self.objects_in_scene):
            points = obj.xyz_points
            # print("Object ", i)
            # print("Num Points: ", len(points))
            # fig = plt.figure()
            # ax = fig.add_subplot(111, projection='3d')
            # ax.scatter(points[:,0], points[:,1], points[:,2],c = colors.hsv_to_rgb(obj.hsv), s=20)
            # plt.show()
            object_pts = np.array(points)
            object_pts = np.hstack((object_pts, np.atleast_2d(np.ones(len(points))*i).T))
            points_with_label = np.vstack((points_with_label, object_pts))
        points_with_label = points_with_label.T
        CLUSTERED_PC = ClusteredPointcloud(x=points_with_label[0].tolist(), y=points_with_label[1].tolist(), z=points_with_label[2].tolist(), cluster_label=points_with_label[3].tolist())
        return CLUSTERED_PC

    def plot_scene(self, ax=None):
        for obj in self.objects_in_scene:
            if ax != None:
                obj.plot_bounding_box(ax)
        if ax != None:
            if self.hsv is None:
                ax.scatter(self.xyz[:,0], self.xyz[:,1], self.xyz[:,2],c = self.dbscan_labels, s=0.01)
            else:
                # ax.scatter(self.xyz[:,0], self.xyz[:,1], self.xyz[:,2], s=1) #,c = self.dbscan_labels
                ax.scatter(self.xyz[:,0], self.xyz[:,1], self.xyz[:,2],c = colors.hsv_to_rgb(self.hsv), s=10)
            add_image_bounding_pixels(ax, np.array([-0.3, -0.3, 0]), np.array([0.3, 0.3, 0.2]))
                
            plt.legend() 

    def plot_all_clustered_objects(self, ax=None):
        axis = None
        if ax is None:
            fig = plt.figure()
            axis = fig.add_subplot(111, projection='3d')
        else:
            axis = ax
        for obj in self.objects_in_scene:
            axis.scatter(obj.xyz_points[:,0], obj.xyz_points[:,1], obj.xyz_points[:,2], s=0.05)

def get_eps(pointcloud):
    pc = np.array(pointcloud)
    neigh = NearestNeighbors(n_neighbors=2)
    nbrs = neigh.fit(pc)
    distances, indices = nbrs.kneighbors(pc)
    distances = np.sort(distances, axis=0)
    distances = distances[:,1]
    # print(np.max(distances) - np.mean(distances))
    # plt.figure(figsize=(8,5))
    # plt.plot(distances)
    # plt.title('K-distance Graph',fontsize=20)
    # plt.xlabel('Data Points sorted by distance',fontsize=14)
    # plt.ylabel('Epsilon',fontsize=14)
    # plt.show()
    return np.max(distances) - np.mean(distances)


def add_image_bounding_pixels(ax, min_point, max_point):
    xyz = np.array([[min_point[0], min_point[1], max_point[2]]])
    xyz = np.append(xyz, [[min_point[0], max_point[1], max_point[2]]], axis = 0)
    xyz = np.append(xyz, [[max_point[0], min_point[1], max_point[2]]], axis = 0)
    xyz = np.append(xyz, [[max_point[0], max_point[1], max_point[2]]], axis = 0)

    ax.scatter(xyz[:,0], xyz[:,1], xyz[:,2], s=0.0001)