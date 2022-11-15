import numpy as np
from scipy.spatial import ConvexHull
import matplotlib.pyplot as plt
import time
from sklearn.cluster import DBSCAN
from iis_panda_controls.srv import SurfaceFeatures
import rospy
import sensor_msgs.msg as sensor_msgs
import std_msgs.msg as std_msgs
from matplotlib import colors


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

class PointCloudScene:
    def __init__(self, debug=False):
        self.objects_in_scene = []
        self.xyz = None
        self.hsv = None
        self.dbscan_labels = None
        self.DEBUG = debug
        self.bounding_boxes = []
    
    def detect_objects(self, xyz, hsv=None):
        self.hsv = hsv
        #xyz, rgb = plane_removal(xyz, rgb, 10**-2)
        print("Detecting Objects...")
        benchmark = []
        benchmark.append(time.time())
        self.xyz = xyz
        if self.DEBUG == False: 
            self.xyz[:,2] = 1.001 - xyz[:,2]
            self.xyz[:,0] = xyz[:,0]
            self.xyz[:,1] = - xyz[:,1]
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

            for i, rgb_ in zip(dbscan.labels_, hsv):
                objects_color[i].append(rgb_)


        for i in range(len(benchmark)-1):
            print("Milestone ", i , " time: ", benchmark[i+1]-benchmark[i])

        for i, object in enumerate(objects):
            # Checks if panda foot is still in the scene or not
            if np.linalg.norm(np.array(object).mean(axis=0) - MEAN_PANDA_FOOT) > 0.01:
                if hsv is None:
                    self.objects_in_scene.append(PointCloudObject(object))
                else:
                    self.objects_in_scene.append(PointCloudObject(object, objects_color[i]))

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
            print(i)
            self.objects_in_scene[i].compute_surface_normals()

    def plot_scene(self, ax=None):
        for obj in self.objects_in_scene:
            if ax != None:
                obj.plot_bounding_box(ax)
        if ax != None:
            if self.hsv is None:
                ax.scatter(self.xyz[:,0], self.xyz[:,1], self.xyz[:,2],c = self.dbscan_labels, s=0.01)
            else:
                ax.scatter(self.xyz[:,0], self.xyz[:,1], self.xyz[:,2],c = colors.hsv_to_rgb(self.hsv), s=10)
            plt.legend() 



class PointCloudObject:
    # Camera Frame to Panda Frame 
    T = np.matrix([[0, 1, 0, 0.5125],
                   [-1, 0, 0, 0.0175],
                   [0, 0, 1, 0],
                   [0, 0, 0, 1]])

    def __init__(self, object, rgb = None):
        self.xyz_points = np.array(object)
        self.rgb = rgb

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

        self.surface_normals_vector = []
        self.surface_normals_angles = []
        self.priciple_curvatures = []

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
            print(response)
            time.sleep(20)
        except rospy.ServiceException as e:
            print("Service failed %s", e)

            
            
        
            

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


    def plot_surface_normals(self, ax=None, special_list = []):
        soa = np.array(self.surface_normals_vector)
        soa.T[3:] = soa.T[3:]/100
        #X, Y, Z, U, V, W = zip(*soa)
        if ax == None:
            fig = plt.figure()
            ax = fig.add_subplot(111, projection='3d')

        for i, coords in enumerate(soa):
            X, Y, Z, U, V, W = coords
            if i in special_list:
                ax.quiver(X, Y, Z, U, V, W, color="red")
            else:
                ax.quiver(X, Y, Z, U, V, W, color="grey")

    def plot_point_cloud(self, ax=None, marker_size = 20):
        if ax == None:
            fig = plt.figure()
            ax = fig.add_subplot(111, projection='3d')
        
        ax.scatter(self.xyz_points[:,0], self.xyz_points[:,1], self.xyz_points[:,2], s=marker_size)
        

    def change_camera_transformation(self, T):
        self.T = T