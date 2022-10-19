from distutils.log import debug
from operator import index
import numpy as np
from scipy.spatial import ConvexHull
import matplotlib.pyplot as plt
from sklearn.neighbors import NearestNeighbors
import time
from sklearn.cluster import DBSCAN

'''
This class describes an object according to the paper Learning Social Affordances and Using Them for Planning 
(https://escholarship.org/content/qt9cj412wg/qt9cj412wg.pdf) by Uyanik et. al (2013)
The properties are divided into surface and spatial features. 
- Spatal features are bounding box pose (x, y, z, theta) and bounding box dimensions (x, y, z)

- Surface features are descibed as surface normals (azimuth and zenith), principal curvatures (min and max) and
    shape indices. They are represented as a 20-bin histogram in addition to the min, max, mean, standard deviation 
    and variance information.
'''

class PointCloudScene:
    def __init__(self, debug=False):
        self.objects_in_scene = []
        self.xyz = None
        self.rgb = None
        self.dbscan_labels = None
        self.DEBUG = debug
    
    def create_bounding_boxes(self, xyz, rgb=None):
        benchmark = []
        benchmark.append(time.time())
        self.xyz = xyz
        if self.DEBUG == False: 
            self.xyz[:,2] = 1.001 - xyz[:,2]  #
            self.xyz[:,0] = xyz[:,0]
            self.xyz[:,1] = - xyz[:,1]
        benchmark.append(time.time())

        # --- Segmentation ---  
        #xyz, rgb = plane_removal(xyz, rgb, 10**-2)
        #benchmark.append(time.time())
        # print(type(self.xyz.dtype))
        # print(self.xyz.shape)
        # print(np.max(self.xyz[:,2]))
        # print(np.min(self.xyz[:,2]))
        dbscan = DBSCAN(eps=0.022, min_samples=6)
        dbscan.fit(self.xyz)
        self.dbscan_labels = dbscan.labels_

        labels_set = set(dbscan.labels_)
        benchmark.append(time.time())

        benchmark.append(time.time())
        objects = []
        for i in range(len(labels_set)):
            objects.append([])

        for i, xyz_ in zip(dbscan.labels_, self.xyz):
            objects[i].append(xyz_)
        benchmark.append(time.time())

        object_bounding_boxes = []
        # --- Bounding Boxes ---
        for object in objects:
            self.objects_in_scene.append(PointCloudObject(object))
            self.objects_in_scene[-1].compute_bounding_box()

            print(self.objects_in_scene[-1].pose() + self.objects_in_scene[-1].size())
            object_bounding_boxes.append(list(self.objects_in_scene[-1].pose()) + self.objects_in_scene[-1].size())
        benchmark.append(time.time())

        for i in range(len(benchmark)-1):
            print("Milestone ", i , " time: ", benchmark[i+1]-benchmark[i])

        
        return object_bounding_boxes
    
    def plot_scene(self, ax=None):
        for obj in self.objects_in_scene:
            if ax != None:
                obj.plot_bounding_box(ax)
        if ax != None:
            ax.scatter(self.xyz[:,0], self.xyz[:,1], self.xyz[:,2],c = self.dbscan_labels, s=0.01)
            plt.legend()
            plt.show()  



class PointCloudObject:
    # Camera Frame to Panda Frame 
    T = np.matrix([[0, 1, 0, 0.5125],
                   [-1, 0, 0, 0.0175],
                   [0, 0, 1, 0],
                   [0, 0, 0, 1]])

    def __init__(self, object):
        self.xyz_points = object

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
        undirected_normals = []
        nbrs = NearestNeighbors(n_neighbors=6, algorithm='ball_tree').fit(self.xyz_points)
        distances, indices = nbrs.kneighbors(self.xyz_points)
        for index_group in indices:
            COV = np.cov(np.array(self.xyz_points)[index_group].T)
            w, v = np.linalg.eig(COV)

            min_ev_index = np.argmin(w)
            undirected_normals.append([*list(self.xyz_points[index_group[0]]),*list(v[min_ev_index]/10000)])

        for un in undirected_normals:
            pointcloudpoint_to_center = np.linalg.norm(np.array(un[:3]) - np.array(self.pose_camera()[:3]))
            vector_tip_to_center = np.linalg.norm((np.array(un[:3]) + np.array(un[3:])) - np.array(self.pose_camera()[:3]))
            if vector_tip_to_center < pointcloudpoint_to_center:
                self.surface_normals_vector.append([*un[:3],*list((-10000) * np.array(un[3:]))])
            else:
                self.surface_normals_vector.append([*un[:3],*list(10000 * np.array(un[3:]))])

        for vec in self.surface_normals_vector:
            zenith = np.arctan2(vec[2], vec[0])
            azimuth = None
            if vec[0] >= 0 and vec[1] >= 0:
                azimuth = np.arctan2(vec[1], vec[0])
            elif vec[0] < 0 and vec[1] >= 0:
                azimuth = np.arctan2(vec[1], vec[0])
            elif vec[0] <= 0 and vec[1] < 0:
                azimuth = np.arctan2(-vec[1], -vec[0]) + np.pi
            elif vec[0] >= 0 and vec[1] < 0:
                azimuth = np.arctan2(-vec[1], vec[0]) + (3*np.pi)/2
            self.surface_normals_angles.append(np.array([azimuth, zenith]))


        
        for i, p in enumerate(self.xyz_points):
            # estimate normal curvatures k_n
            k_n = []
            theta_n = []
            M_n = []
            first_index = True
            for index_nbr in indices[i]:
                if first_index == True:
                    first_index = False
                    continue
                q_index = index_nbr #indices[i][1]
                q = self.xyz_points[q_index] - p
                M_i = self.surface_normals_vector[q_index][3:]
            
                n_xy = (q[0]*M_i[0] + q[1]*M_i[1]) / np.sqrt(q[0]**2 + q[1]**2)

                k_n.append(n_xy / (np.sqrt(n_xy**2 + M_i[2]**2)*np.sqrt(q[0]**2 + q[1]**2)))

                #Least Square Fitting for principle curvatures
                N = self.surface_normals_vector[i][3:]
                psi = np.arccos(N[2])
                phi = np.arctan2(N[1], N[0])
                X = np.array([-np.sin(phi), np.cos(phi), 0])
                Y = np.array([np.cos(psi)*np.cos(phi), np.cos(psi)*np.sin(phi), -np.sin(psi)])
                # print(N)
                # print(X)
                # print(Y)

                # project pq onto Plane X-Y
                proj_pq_on_N = np.dot(p*q, N)*np.array(N)
                proj_pq_on_XY = p*q - proj_pq_on_N
                theta = np.arccos(np.dot(X, proj_pq_on_XY/np.linalg.norm(proj_pq_on_XY))) 
                theta_n.append(theta)
                M_n.append(np.array([np.cos(theta)**2, 2*np.sin(theta)*np.cos(theta), np.sin(theta)**2]))

            M = np.array(M_n, dtype=np.float)
            k_n = np.array(k_n, dtype=np.float)
            A, B, C = np.linalg.lstsq(M, k_n, rcond=-1)[0]
            weingarten_matrix = np.matrix([[A, B], [B, C]])
            
            principal_cuvatures = np.linalg.eigvalsh(weingarten_matrix)
            self.priciple_curvatures.append(principal_cuvatures)
        

            
            # soa = np.array([[0,0,0,*X], [0,0,0,*Y],[0,0,0,*(proj_pq_on_XY/np.linalg.norm(proj_pq_on_XY))],[0,0,0,*((p*q)/np.linalg.norm(p*q))]])
            # soa.T[3:] = soa.T[3:]/100

            # soas = [[0,0,0,*X], [0,0,0,*Y],[0,0,0,*(proj_pq_on_XY/np.linalg.norm(proj_pq_on_XY))],[0,0,0,*((p*q)/np.linalg.norm(p*q))]]
            # labels = ["X", "Y", "proj_XY", "pq"]
            # colors = ["red", "green", "blue", "yellow"]
            # fig = plt.figure()
            # ax = fig.add_subplot(111, projection='3d')
            # for i,soa in enumerate(soas):
            #     X, Y, Z, U, V, W = soa #zip(*soa)
            #     ax.quiver(X, Y, Z, U, V, W, label=labels[i], color=colors[i])
            # plt.legend()
            # plt.show()
            # if i > 3:
            #     exit()

            
            
        
            

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

    def change_camera_transformation(self, T):
        self.T = T