import numpy as np
from scipy.spatial import ConvexHull,Delaunay
from matplotlib.path import Path
import matplotlib.pyplot as plt
import time
from sklearn.cluster import DBSCAN
from sklearn.preprocessing import normalize
from elsa_perception_msgs.srv import SurfaceFeatures
from elsa_object_database.srv import RegisteredObjects
from elsa_perception_msgs.msg import PhysicalScene, PhysicalFeatures, BoundingBox, SurfaceFeaturesMsg, ClusteredPointcloud

import rospy
import sensor_msgs.msg as sensor_msgs
import std_msgs.msg as std_msgs
from matplotlib import colors
import copy
from itertools import combinations
from perception.real_world_preprocessing import remove_plane

'''
This class describes an object according to the paper Learning Social Affordances and Using Them for Planning 
(https://escholarship.org/content/qt9cj412wg/qt9cj412wg.pdf) by Uyanik et. al (2013)
The properties are divided into surface and spatial features. 
- Spatal features are bounding box pose (x, y, z, theta) and bounding box dimensions (x, y, z)

- Surface features are descibed as surface normals (azimuth and zenith), principal curvatures (min and max) and
    shape indices. They are represented as a 20-bin histogram in addition to the min, max, mean, standard deviation 
    and variance information.
'''

MEAN_PANDA_FOOT = np.array([ 0.01781263, -0.46853645,  0.04416075])

COLOR_SEGMENTATION = True

def add_image_bounding_pixels(ax, min_point, max_point):
    xyz = np.array([[min_point[0], min_point[1], max_point[2]]])
    xyz = np.append(xyz, [[min_point[0], max_point[1], max_point[2]]], axis = 0)
    xyz = np.append(xyz, [[max_point[0], min_point[1], max_point[2]]], axis = 0)
    xyz = np.append(xyz, [[max_point[0], max_point[1], max_point[2]]], axis = 0)

    ax.scatter(xyz[:,0], xyz[:,1], xyz[:,2], s=0.0001)

class PointCloudScene:
    def __init__(self, debug=False):
        self.objects_in_scene = []
        self.xyz = None
        self.hsv = None
        self.dbscan_labels = None
        self.DEBUG = debug
        self.bounding_boxes = []
        
        self.registered_objs = []
        self.registered_objs_values = np.empty((2,))
        self.registered_obj_client()        

        if debug is False:
            self.color_eps = 0.075
        else:
            self.color_eps = 0.2

    
    def detect_objects(self, xyz, hsv=None):
        self.hsv = hsv

        print("Detecting Objects...")
        benchmark = []
        benchmark.append(time.time())
        self.xyz = xyz
        if self.DEBUG == False: 
            self.xyz[:,2] = 0.946 - xyz[:,2]
            self.xyz[:,0] = xyz[:,0]
            self.xyz[:,1] = - xyz[:,1]

            self.xyz, self.hsv = remove_plane(self.xyz, hsv, 0.01)


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

        # fig = plt.figure()
        # ax = fig.add_subplot(111, projection='3d')
        # ax.scatter(self.xyz[:,0], self.xyz[:,1], self.xyz[:,2],c = colors.hsv_to_rgb(self.hsv), s=20) 
        # plt.show()

        for i in range(len(benchmark)-1):
            print("Milestone ", i , " time: ", benchmark[i+1]-benchmark[i])

        for i, obj in enumerate(objects):
            # Checks if panda foot is still in the scene or not
            if np.linalg.norm(np.array(obj).mean(axis=0) - MEAN_PANDA_FOOT) > 0.01:
                if len(obj) > 10:
                    if hsv is None:
                        self.objects_in_scene.append(PointCloudObject(obj))
                    else:
                        self.objects_in_scene.append(PointCloudObject(obj, objects_color[i]))
        if hsv is not None and COLOR_SEGMENTATION == True:
            old_objects_in_scene = copy.copy(self.objects_in_scene)        
            for whole_obj_indx, obj in enumerate(old_objects_in_scene):
                # print("Object ", whole_obj_indx, ":")
                xyz_object = normalize(np.array(obj.xyz_points)) 

                # Parametrize h value to circle in for cyclical clustering
                h_value = np.array(obj.hsv)[:,0] * 2*np.pi
                s_value = np.array(obj.hsv)[:,1]
                v_value = np.array(obj.hsv)[:,2]
                # np.save("/home/marko/saved_arrays/h_values_bad.npy", h_value)
                h_x = np.sin(h_value)
                h_y = np.cos(h_value)
                h_circle = np.concatenate((np.atleast_2d(h_x).T, np.atleast_2d(h_y).T), axis=1) #, xyz_object/20)
                cluster = DBSCAN(eps=self.color_eps, min_samples=8) 
                cluster.fit(h_circle)

                # print("colors detected: ", len(set(cluster.labels_)))

                # If the clustering algorithm detects more than one object calculate certainty measure
                list_of_labels = list(set(cluster.labels_))
                colors_detected = len(set(cluster.labels_))
                print("Color clusters in object: ", colors_detected)
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
                   
                   
                    # list_indices = range(len(new_objs))
                    # combos = list(combinations(list_indices, 2))
                
                  
                    self.objects_in_scene.pop(whole_obj_indx)
                    new_objs_copy = new_objs.copy()
                    list_of_small_objs = []
                    for indx in range(len(set(cluster.labels_))):
                        if len(new_objs_copy[indx]) < 30:
                            list_of_small_objs.append(indx)

                    list_of_small_objs.reverse()
                    print("Small objects: ", list_of_small_objs)
                    for i in list_of_small_objs:
                        new_objs.pop(i)
                        new_objs_colors.pop(i)
                    

                    for indx in range(len(new_objs)):
                        ob_array = np.array(new_objs[indx])
                        q75_x,q25_x = np.percentile(ob_array[:,0],[75,25])
                        q75_y,q25_y = np.percentile(ob_array[:,1],[75,25])
                        q75_z,q25_z = np.percentile(ob_array[:,2],[75,25])
                        intr_qr_x = q75_x-q25_x
                        max_x = q75_x+(1.5*intr_qr_x)
                        min_x = q25_x-(1.5*intr_qr_x)
                        intr_qr_y = q75_y-q25_y
                        max_y = q75_y+(1.5*intr_qr_y)
                        min_y = q25_y-(1.5*intr_qr_y)
                        intr_qr_z = q75_x-q25_x
                        max_z = q75_z+(1.5*intr_qr_z)
                        min_z = q25_z-(1.5*intr_qr_z)
                        
                        for i_pt in range(len(new_objs[indx])-1,-1,-1):
                            pt = new_objs[indx][i_pt]
                            if min_x > pt[0] or max_x < pt[0]: 
                                new_objs[indx].pop(i_pt)
                                new_objs_colors[indx].pop(i_pt)
                            elif min_y > pt[1] or max_y < pt[1]:
                                new_objs[indx].pop(i_pt)
                                new_objs_colors[indx].pop(i_pt)
                            elif min_z > pt[2] or max_z < pt[2]:
                                new_objs[indx].pop(i_pt)
                                new_objs_colors[indx].pop(i_pt)

                        self.objects_in_scene.append(PointCloudObject(new_objs[indx], new_objs_colors[indx]))
        
        # self.create_bounding_boxes()
        # for obj in self.objects_in_scene:
        #     print(len(obj.xyz_points))
        #     print(len(obj.hsv))
        #     fig = plt.figure()
        #     axis = fig.add_subplot(111, projection='3d')
        #     add_image_bounding_pixels(axis, np.array([-0.3, -0.3, 0]), np.array([0.3, 0.3, 0.3]))
        #     axis.scatter(obj.xyz_points[:,0], obj.xyz_points[:,1], obj.xyz_points[:,2], s=10,c = colors.hsv_to_rgb(obj.hsv))
        #     obj.plot_bounding_box(axis)
        #     plt.show()
   
            

    def create_bounding_boxes(self):
        print("Computing Bounding boxes ...")
        benchmark = []

        object_bounding_boxes = []
        benchmark.append(time.time())
        for i in range(len(self.objects_in_scene)):
            self.objects_in_scene[i].compute_bounding_box()
            print(self.objects_in_scene[i].pose() + self.objects_in_scene[i].size())
            object_bounding_boxes.append(list(self.objects_in_scene[i].pose()) + self.objects_in_scene[i].size())
        benchmark.append(time.time())

        for i in range(len(benchmark)-1):
            print("Milestone ", i , " time: ", benchmark[i+1]-benchmark[i])

        self.bounding_boxes = object_bounding_boxes
        return object_bounding_boxes
    
    def calculate_surface_features(self):
        print("Computing Surface features ...")
        # --- Surface Features ---
        for i in range(len(self.objects_in_scene)):
            self.objects_in_scene[i].compute_surface_normals()

    def create_physical_scene_msg(self):
        
        list_physical_features = []

        for obj in self.objects_in_scene:
            list_physical_features.append(obj.create_physical_features_msg(self.registered_objs, self.registered_objs_values))

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
            print("Object ", i)
            print("Num Points: ", len(points))
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




class PointCloudObject:
    # Camera Frame to Panda Frame 
    T = np.matrix([[0, 1, 0, 0.5125],
                   [-1, 0, 0, 0.0175],
                   [0, 0, 1, 0],
                   [0, 0, 0, 1]])

    def __init__(self, object, hsv = None):
        self.xyz_points = np.array(object)
        self.hsv = hsv

        # Object Pose Camera
        self.center_x_camera = 0
        self.center_y_camera = 0
        self.center_z_camera = 0
        self.theta_camera = 0

        # Object pose
        self.center_x = 0
        self.center_y = 0
        self.center_z = 0
        self.theta = 0

        # Object Size
        self.size_x = 0
        self.size_y = 0
        self.size_z = 0

        self.normal_azimut = None
        self.normal_zenith = None
        self.min_curvature = None
        self.max_curvature = None
        self.shape_index = None

    def compute_bounding_box(self):
        self.is_circular = self.__is_object_circular__(self.xyz_points)

        pose_fixed_frame = [0, 0, 0, 0]

        points = np.array(self.xyz_points)

        min_z = 0 #np.min(points[:,2])
        max_z = np.max(points[:,2])
        min_x = np.min(points[:,0])
        min_y = np.min(points[:,1])
        max_x = np.max(points[:,0])
        max_y = np.max(points[:,1])
        x_diff = max_x - min_x
        y_diff = max_y - min_y
        self.center_x_camera = min_x + x_diff/2
        self.center_y_camera = min_y + y_diff/2
        z_diff = max_z - min_z

        # pose z 
        self.center_z_camera = min_z + z_diff/2
        theta = None
        theta2 = 0.0
        
        if self.is_circular:
            print("It is circular!!!!")
            theta = 0

            # determine center
            min_x = np.min(points[:,0])
            min_y = np.min(points[:,1])
            max_x = np.max(points[:,0])
            max_y = np.max(points[:,1])

            x_diff = max_x - min_x
            y_diff = max_y - min_y

            self.center_x_camera = min_x + x_diff/2
            self.center_y_camera = min_y + y_diff/2

        else:
            # get theta 
            points2D = points[:,0:2] 
            try:
                hull = ConvexHull(points2D)
            except:
                self.center_x = 1000
                self.center_y = 1000
                self.center_z = 1000

                # store orientation
                self.theta_camera = 1000
                self.theta = 1000

                # dimensions
                self.size_x = x_diff
                self.size_y = y_diff
                self.size_z = z_diff
                return 
            verts = hull.vertices
            max_dist = 0
            max_edge = None

            for simplex in hull.simplices:
                diff = points2D[simplex][0] - points2D[simplex][1]
                dist = np.linalg.norm(diff)
                if dist > max_dist:
                    max_dist = dist
                    max_edge = points2D[simplex][0] - points2D[simplex][1]

            edge = max_edge/np.linalg.norm(max_edge)
            if edge[0] >= 0 and edge[1] >= 0:
                x = np.array([0, 1])
            elif edge[0] >= 0 and edge[1] < 0:
                x = np.array([1, 0])
            elif edge[0] < 0 and edge[1] < 0:
                x = np.array([0, -1])
            else:
                x = np.array([-1, 0])

            theta = np.arccos(np.dot(x,edge)) 

            orig = np.random.randn(2)
            orig -= orig.dot(edge) * edge
            orig /= np.linalg.norm(orig)

            if orig[0] >= 0 and orig[1] >= 0:
                x = np.array([0, 1])
            elif orig[0] >= 0 and orig[1] < 0:
                x = np.array([1, 0])
            elif orig[0] < 0 and orig[1] < 0:
                x = np.array([0, -1])
            else:
                x = np.array([-1, 0])

            theta2 = np.arccos(np.dot(x,orig)) 

            xy_pose_camera_frame = np.array([self.center_x_camera, self.center_y_camera])
            # # rotate points and get center and dimensions
            R = np.array([[np.cos(theta), -np.sin(theta)], [np.sin(theta), np.cos(theta)]])
            points_rot = np.dot(R, (points2D-xy_pose_camera_frame).T).T + xy_pose_camera_frame

            # determine center
            min_x = np.min(points_rot[:,0])
            min_y = np.min(points_rot[:,1])
            max_x = np.max(points_rot[:,0])
            max_y = np.max(points_rot[:,1])

            x_diff = max_x - min_x
            y_diff = max_y - min_y

        center = np.array(np.array([self.center_x_camera, self.center_y_camera, self.center_z_camera, 1]))
        pose_fixed_frame = np.ravel(np.matmul(self.T, center))

        self.center_x = pose_fixed_frame[0]
        self.center_y = pose_fixed_frame[1]
        self.center_z = pose_fixed_frame[2]

        # store orientation
        self.theta_camera = theta
        self.theta = np.pi/2 - theta2
        if self.is_circular:
            self.theta = 0 
            
        # dimensions
        self.size_x = x_diff
        self.size_y = y_diff
        self.size_z = z_diff

    def compute_surface_normals(self):
        # convert numpy array into sensor_msgs/PointCloud2 
        ros_dtype = sensor_msgs.PointField.FLOAT32
        dtype = np.float32
        itemsize = np.dtype(dtype).itemsize
        rgb_values = None
        # if self.hsv is None:
        rgb_values = np.ones_like(self.xyz_points) * 0.6
        # else:
        #     rgb_values = self.hsv
        
        hex_rgb = []
        for rgb_ in rgb_values:
            hex_rgb.append(np.float32(int(colors.rgb2hex(rgb_).replace('#', '0x'), 0)))
        hex_rgb = np.atleast_2d(np.array(hex_rgb)).T
        points = np.hstack((self.xyz_points, hex_rgb))

        data = points.astype(dtype).tobytes()

        fields = [sensor_msgs.PointField(
        name=n, offset=i*itemsize, datatype=ros_dtype, count=1)
        for i, n in enumerate(['x', 'y', 'z', 'rgb'])]

        header = std_msgs.Header(frame_id="/map", stamp=rospy.Time.now())
        srv_input = sensor_msgs.PointCloud2(header=header,
                                height=1,
                                width=points.shape[0],
                                is_dense=False,
                                is_bigendian=False,
                                fields=fields,
                                point_step=(itemsize * 4),
                                row_step=(itemsize * 4 * points.shape[0]),
                                data=data
        )
        rospy.wait_for_service("calculate_surface_features")
        try:
            srv_calc_surface_features = rospy.ServiceProxy("calculate_surface_features", SurfaceFeatures)
            response = srv_calc_surface_features(srv_input)
            self.normal_azimut = response.surface_features.features[0] 
            self.normal_zenith = response.surface_features.features[1]
            self.min_curvature = response.surface_features.features[2]
            self.max_curvature = response.surface_features.features[3]
            self.shape_index = response.surface_features.features[4]
            
        except rospy.ServiceException as e:
            print("Service failed %s", e)

    def create_physical_features_msg(self, registered_obj_names, registered_obj_values):
        physical_features = PhysicalFeatures()
        bounding_box = BoundingBox()
        surface_features = SurfaceFeaturesMsg()

        bounding_box.x = self.center_x
        bounding_box.y = self.center_y
        bounding_box.z = self.center_z
        bounding_box.phi = self.theta
        bounding_box.dx = self.size_x 
        bounding_box.dy = self.size_y 
        bounding_box.dz = self.size_z 

        surface_features.normal_azimut = self.normal_azimut 
        surface_features.normal_zenith = self.normal_zenith 
        surface_features.min_curvature = self.min_curvature 
        surface_features.max_curvature = self.max_curvature 
        surface_features.shape_index   = self.shape_index   
        
        physical_features.spatial_features = bounding_box
        physical_features.surface_features = surface_features

        if self.hsv == None:
            physical_features.mean_color = [-1,-1,-1] 
        else:
            hsv_mean = np.mean(self.hsv, axis=0)
            physical_features.mean_color = hsv_mean
            x = np.cos(np.pi*2*hsv_mean[0])
            y = np.sin(np.pi*2*hsv_mean[0])


            total = np.abs(registered_obj_values - np.array([[x,y]]))
            label_index = np.sum(total,axis=1).argmin()

            # debugging non-unicity of object class (color)
            #rospy.logerr(registered_obj_names[label_index])
            #rospy.logerr(label_index)
            physical_features.obj_identity = registered_obj_names[label_index]

        return physical_features

            

    def __is_object_circular__(self, object):
        #+++ check how circular an object is? Rotation does not effect these objects +++
        points = np.array(object)[:,0:2] 
        center = [0, 0]
        dimensions = [0, 0]
        min_x = np.min(points[:,0])
        min_y = np.min(points[:,1])
        max_x = np.max(points[:,0])
        max_y = np.max(points[:,1])

        # dimensions
        dimensions[0] = max_x - min_x
        dimensions[1] = max_y - min_y

        #pose
        center[0] = min_x + dimensions[0]/2
        center[1] = min_y + dimensions[1]/2
        total_diff = 0
        dims = []
        for deg in [np.pi/4, np.pi/8, np.pi/16, -np.pi/8, -np.pi/16]:
            W = np.array([[np.cos(deg), np.cos(deg + np.pi)], [np.sin(deg), np.sin(deg + np.pi)]])
            proj_points = points.dot(W)
            dimensions2 = [0, 0]
            min_x = np.min(proj_points[:,0])
            min_y = np.min(proj_points[:,1])
            max_x = np.max(proj_points[:,0])
            max_y = np.max(proj_points[:,1])

            # dimensions
            dimensions2[0] = max_x - min_x
            dimensions2[1] = max_y - min_y

            total_diff += np.sum(np.abs(np.array(dimensions) - np.array(dimensions2)))
            dims.append(dimensions2)

        for dim in dims:
            for dim2 in dims:
                total_diff += np.sum(np.abs(np.array(dim) - np.array(dim2)))

        #print("Total diff: ", total_diff)
        return total_diff < 0.22

    def pose_camera(self):
        return [self.center_x_camera, self.center_y_camera, self.center_z_camera, self.theta_camera]


    def pose(self):
        return [self.center_x, self.center_y, self.center_z, self.theta]

    def size(self):
        return [self.size_x, self.size_y, self.size_z]

    def plot_bounding_box(self, ax=None):
        # Calculate Cornerpoints
        unrotated_points = []
        unrotated_points.append(np.array([self.center_x_camera + (self.size_x/2), self.center_y_camera + (self.size_y/2), self.center_z_camera + (self.size_z/2)])) #p_left_up_top    
        unrotated_points.append(np.array([self.center_x_camera - (self.size_x/2), self.center_y_camera + (self.size_y/2), self.center_z_camera + (self.size_z/2)])) #p_right_up_top   
        unrotated_points.append(np.array([self.center_x_camera + (self.size_x/2), self.center_y_camera - (self.size_y/2), self.center_z_camera + (self.size_z/2)])) #p_left_down_top  
        unrotated_points.append(np.array([self.center_x_camera - (self.size_x/2), self.center_y_camera - (self.size_y/2), self.center_z_camera + (self.size_z/2)])) #p_right_down_top 
        unrotated_points.append(np.array([self.center_x_camera + (self.size_x/2), self.center_y_camera + (self.size_y/2), self.center_z_camera - (self.size_z/2)])) #p_left_up_bottom
        unrotated_points.append(np.array([self.center_x_camera - (self.size_x/2), self.center_y_camera + (self.size_y/2), self.center_z_camera - (self.size_z/2)])) #p_right_up_bottom
        unrotated_points.append(np.array([self.center_x_camera + (self.size_x/2), self.center_y_camera - (self.size_y/2), self.center_z_camera - (self.size_z/2)])) #p_left_down_bottom
        unrotated_points.append(np.array([self.center_x_camera - (self.size_x/2), self.center_y_camera - (self.size_y/2), self.center_z_camera - (self.size_z/2)])) #p_right_down_bottom
        unrotated_points = np.array(unrotated_points)
        theta = -self.theta_camera
        xyz = np.array([self.center_x_camera, self.center_y_camera, self.center_z_camera])
        R = np.array([[np.cos(theta), -np.sin(theta), 0], [np.sin(theta), np.cos(theta), 0], [0, 0, 1]])
        corner_points = np.dot(R, (unrotated_points-xyz).T).T + xyz
        
        if ax == None:
          ax = plt.axes(projection='3d')  
        ax.scatter(corner_points[:,0], corner_points[:,1], corner_points[:,2], marker='^')


    # def plot_surface_normals(self, ax=None, special_list = []):
    #     soa = np.array(self.surface_normals_vector)
    #     soa.T[3:] = soa.T[3:]/100
    #     #X, Y, Z, U, V, W = zip(*soa)
    #     if ax == None:
    #         fig = plt.figure()
    #         ax = fig.add_subplot(111, projection='3d')

    #     for i, coords in enumerate(soa):
    #         X, Y, Z, U, V, W = coords
    #         if i in special_list:
    #             ax.quiver(X, Y, Z, U, V, W, color="red")
    #         else:
    #             ax.quiver(X, Y, Z, U, V, W, color="grey")

    def plot_point_cloud(self, ax=None, marker_size = 20):
        if ax == None:
            fig = plt.figure()
            ax = fig.add_subplot(111, projection='3d')
        
        ax.scatter(self.xyz_points[:,0], self.xyz_points[:,1], self.xyz_points[:,2], s=marker_size)
        

    def change_camera_transformation(self, T):
        self.T = T


# def in_hull(p, hull):
#     """
#     Test if points in `p` are in `hull`

#     `p` should be a `NxK` coordinates of `N` points in `K` dimensions
#     `hull` is either a scipy.spatial.Delaunay object or the `MxK` array of the 
#     coordinates of `M` points in `K`dimensions for which Delaunay triangulation
#     will be computed
#     """
#     if not isinstance(hull,Delaunay):
#         hull = Delaunay(hull)

#     return hull.find_simplex(p)>=0

def in_hull(test_points, hull_object):
    # Assume points are shape (n, d), and that hull has f facets.
    hull = ConvexHull(hull_object)
    print("Volume ", hull.volume)
    # A is shape (f, d) and b is shape (f, 1).
    A, b = hull.equations[:, :-1], hull.equations[:, -1:]
    eps = np.finfo(np.float32).eps
    # The hull is defined as all points x for which Ax + b <= 0.
    # We compare to a small positive value to account for floating
    # point issues.
    #
    # Assuming x is shape (m, d), output is boolean shape (m,).
    return np.all(np.asarray(test_points) @ A.T + b.T < eps, axis=1)


'''
The certainty measure is calculated by how much of one object is found in the other:
1.) calculate the convex hull of all the objects 
2.) check objects pairwise of how many points of each object are in the other one
3.) calculate the ratio of object points to foreign points -> certainty measure
'''
def certainty_measure_color(obj1, obj2):
    if len(obj1) >= 5 and len(obj2) >= 5:
        unique, counts = np.unique(in_hull(obj2, obj1), return_counts=True)
        unique2, counts2 = np.unique(in_hull(obj1, obj2), return_counts=True)
        stat = dict(zip(unique, counts))
        stat2 = dict(zip(unique2, counts2))
        print(stat)
        print(stat2)
        forein_ratio_obj1 = 0
        forein_ratio_obj2 = 0
        if True in stat:
            forein_ratio_obj1 = stat[True] / len(obj1)
        if True in stat2:
            forein_ratio_obj2 = stat2[True] / len(obj2)
        return 1 - (loss_function(forein_ratio_obj1)*0.5 + loss_function(forein_ratio_obj2)*0.5)
    else:
        return 0
    
def loss_function(x):
    return (1-np.exp(-x * 1000))