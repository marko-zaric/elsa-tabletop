import numpy as np
from scipy.spatial import ConvexHull
import matplotlib.pyplot as plt
from elsa_perception_msgs.srv import SurfaceFeatures
from elsa_perception_msgs.msg import PhysicalFeatures, BoundingBox, SurfaceFeaturesMsg
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

    def create_physical_features_msg(self, registered_obj_names, registered_obj_values, register_objects=False):
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


            if register_objects == True:
                total = np.abs(registered_obj_values - np.array([[x,y]]))
                label_index = np.sum(total,axis=1).argmin()
                # debugging non-unicity of object class (color)
                #rospy.logerr(registered_obj_names[label_index])
                #rospy.logerr(label_index)
                physical_features.obj_identity = registered_obj_names[label_index]
            else:
                physical_features.obj_identity = "Unregistered Object"

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


