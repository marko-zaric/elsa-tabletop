import numpy as np

def remove_plane(xyz, color, tol):
    xyz_planeless = []
    rgb_planeless = []

    # Background detection
    # table_z = min(xyz[:,2][np.nonzero(xyz[:,2])])
    
    # plane removal

    min_z = min(xyz[:,2])
    for(xyz_, hsv_) in zip(xyz, color):
        if hsv_[1] > 0.4 and hsv_[2] < 0.8:
            if abs(xyz_[2] - min_z) > tol:
                if xyz_[2] < 0.3:
                    xyz_planeless.append(xyz_)
                    rgb_planeless.append(hsv_)
        
    xyz_planeless = np.array(xyz_planeless)
    rgb_planeless = np.array(rgb_planeless)
    return xyz_planeless, rgb_planeless