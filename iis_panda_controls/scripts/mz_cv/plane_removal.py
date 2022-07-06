import numpy as np

def plane_removal(xyz, rgb):
    xyz_planeless = []
    rgb_planeless = []

    # Background detection
    # table_z = min(xyz[:,2][np.nonzero(xyz[:,2])])
    # rgb_distances = np.sqrt((rgb[:,0])**2 + (rgb[:,1])**2 + (rgb[:,2])**2)
    # bins = np.histogram(rgb_distances, bins=1000)
    # background = bins[1][np.argmax(bins[0])]
    
    # plane removal
    mean=np.mean(xyz, axis=0)
    for(xyz_, rgb_) in zip(xyz, rgb):
        #if abs(background - rgb_dist(rgb_)) < 2:
        if abs(xyz_[2] - mean[2]) > 10**-2:
            xyz_planeless.append(xyz_)
            rgb_planeless.append(rgb_)

    xyz_planeless = np.array(xyz_planeless)
    rgb_planeless = np.array(rgb_planeless)
    return xyz_planeless, rgb_planeless

def get_image_corners(xyz):
    min_x = np.min(xyz[:,0])
    min_y = np.min(xyz[:,1])
    min_z = np.min(xyz[:,2])
    max_x = np.max(xyz[:,0])
    max_y = np.max(xyz[:,1])
    max_z = 0.5 #np.max(xyz[:,2])
    min_point = [min_x, min_y, min_z]
    max_point = [max_x, max_y, max_z] 
    return min_point, max_point

def rgb_dist(rgb_vec):
    return np.sqrt((rgb_vec[0])**2 + (rgb_vec[1])**2 + (rgb_vec[2])**2)