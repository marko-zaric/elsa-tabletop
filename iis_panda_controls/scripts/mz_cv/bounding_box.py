import numpy as np
import itertools
from sklearn.decomposition import PCA
from scipy.spatial import ConvexHull
from sklearn.cluster import DBSCAN

# Camera Frame to Panda Frame 
T = np.matrix([[0, 1, 0, 0.5125],
               [-1, 0, 0, 0.0175],
               [0, 0, 1, 0],
               [0, 0, 0, 1]])


def is_object_circular(obj):
#+++ check how circular an object is? Rotation does not effect these objects +++
    points = np.array(obj)[:,0:2] 
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
    return total_diff < 0.2

def convex_hull_bounding_box(obj):
    pose_camera_frame = [0, 0, 0, 0] # x,y,z of center of bounding box; theta
    pose_fixed_frame = [0, 0, 0, 0]
    dimensions = [0, 0, 0] # x,y,z dimensions
    
    points = np.array(obj)
    
    
    min_z = np.min(points[:,2])
    max_z = np.max(points[:,2])
    min_x = np.min(points[:,0])
    min_y = np.min(points[:,1])
    max_x = np.max(points[:,0])
    max_y = np.max(points[:,1])
    x_diff = max_x - min_x
    y_diff = max_y - min_y
    pose_camera_frame[0] = min_x + x_diff/2
    pose_camera_frame[1] = min_y + y_diff/2
    z_diff = max_z - min_z

    # pose z 
    pose_camera_frame[2] = min_z + z_diff/2
    theta = None

    if is_object_circular(obj):
        theta = 0

        # determine center
        min_x = np.min(points[:,0])
        min_y = np.min(points[:,1])
        max_x = np.max(points[:,0])
        max_y = np.max(points[:,1])

        x_diff = max_x - min_x
        y_diff = max_y - min_y

        pose_camera_frame[0] = min_x + x_diff/2
        pose_camera_frame[1] = min_y + y_diff/2

    else:
        # get theta 
        points2D = points[:,0:2] 
        hull = ConvexHull(points2D)
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
        

        # # rotate points and get center and dimensions
        R = np.array([[np.cos(theta), -np.sin(theta)], [np.sin(theta), np.cos(theta)]])
        points_rot = np.dot(R, (points2D).T).T

        # determine center
        min_x = np.min(points_rot[:,0])
        min_y = np.min(points_rot[:,1])
        max_x = np.max(points_rot[:,0])
        max_y = np.max(points_rot[:,1])

        x_diff = max_x - min_x
        y_diff = max_y - min_y

    # store orientation
    pose_camera_frame[3] = theta

    # dimensions
    dimensions[0] = x_diff
    dimensions[1] = y_diff
    dimensions[2] = max_z

    return (pose_camera_frame, dimensions)
    



def bounding_box_diagonals(object, center):
    points_2d = np.array(object)[:,0:2]
    pca = PCA(n_components=2)
    X2D = pca.fit_transform(points_2d)
    #print("Explained Variance ratio: ", pca.explained_variance_ratio_)
    #print(pca.explained_variance_ratio_)
    # print(pca.components_)
    t = np.linspace(-0.2,0.2,100)
    axis1 = np.outer(t, pca.components_[0]) + center
    axis2 = np.outer(t, pca.components_[1]) + center
    return axis1, axis2, pca.components_[0], pca.components_[1]

def bounding_box_spartial_features(object, ax=None):
    pose_camera_frame = [0, 0, 0, 0] # x,y,z of center of bounding box; theta
    pose_fixed_frame = [0, 0, 0, 0]
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
    pose_camera_frame[0] = min_x + dimensions[0]/2
    pose_camera_frame[1] = min_y + dimensions[1]/2
    pose_camera_frame[2] = min_z + dimensions[2]/2

    ax_line, ax_line2, ax1, ax2 = bounding_box_axis(object, [pose_camera_frame[0], pose_camera_frame[1]])

    theta = convex_hull_bounding_box(object, ax)

    x = np.array([1, 0])
    # ax1 = ax1/np.linalg.norm(ax1)
    # ax2 = ax2/np.linalg.norm(ax2)
    # theta = np.arccos(np.dot(x,ax1) / (np.linalg.norm(x) + np.linalg.norm(ax1)))
    # theta = theta % (np.pi/4)

    # Plot PCA Axis
    if ax != None:
        ax.plot(ax_line[:,0], ax_line[:,1], np.ones(len(ax_line[:,1]))*pose_camera_frame[2], 'black')
        ax.plot(ax_line2[:,0], ax_line2[:,1], np.ones(len(ax_line2[:,1]))*pose_camera_frame[2], 'black')



    center = np.array(pose_camera_frame)
    center[3] = 1
    pose_fixed_frame = np.ravel(np.matmul(T, center))
    #pose_fixed_frame[3] = theta
    #print("Pose fixed Frame: ", pose_fixed_frame)

    return (pose_fixed_frame, dimensions)

def add_image_bounding_pixels(ax, min_point, max_point):
    xyz = np.array([[min_point[0], min_point[1], max_point[2]]])
    xyz = np.append(xyz, [[min_point[0], max_point[1], max_point[2]]], axis = 0)
    xyz = np.append(xyz, [[max_point[0], min_point[1], max_point[2]]], axis = 0)
    xyz = np.append(xyz, [[max_point[0], max_point[1], max_point[2]]], axis = 0)

    ax.scatter(xyz[:,0], xyz[:,1], xyz[:,2], s=0.0001)


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


