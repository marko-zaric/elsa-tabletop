import numpy as np
import matplotlib.pyplot as plt
from matplotlib import collections  as mc
import itertools

xyz = np.load("/home/marko/Desktop/IIS_Research/catkin_workspaces/panda_catkin_ws/src/panda_simulator/iis_panda_controls/scripts/xyz.npy")
rgb = np.load("/home/marko/Desktop/IIS_Research/catkin_workspaces/panda_catkin_ws/src/panda_simulator/iis_panda_controls/scripts/rgb.npy")

xyz[:,2] = 1-xyz[:,2]

mean=np.mean(xyz, axis=0)

def rgb_dist(rgb_vec):
    return np.sqrt((rgb_vec[0])**2 + (rgb_vec[1])**2 + (rgb_vec[2])**2)


def add_image_bounding_pixels(ax, min_point, max_point):
    xyz = np.array([[min_point[0], min_point[1], max_point[2]]])
    xyz = np.append(xyz, [[min_point[0], max_point[1], max_point[2]]], axis = 0)
    xyz = np.append(xyz, [[max_point[0], min_point[1], max_point[2]]], axis = 0)
    xyz = np.append(xyz, [[max_point[0], max_point[1], max_point[2]]], axis = 0)
    print(xyz.shape)
    ax.scatter(xyz[:,0], xyz[:,1], xyz[:,2], s=0.0001)


# get frame
min_x = np.min(xyz[:,0])
min_y = np.min(xyz[:,1])
min_z = np.min(xyz[:,2])
max_x = np.max(xyz[:,0])
max_y = np.max(xyz[:,1])
max_z = 0.5 #np.max(xyz[:,2])
min_point = [min_x, min_y, min_z]
max_point = [max_x, min_y, max_z] 


# Background detection
table_z = min(xyz[:,2][np.nonzero(xyz[:,2])])
rgb_distances = np.sqrt((rgb[:,0])**2 + (rgb[:,1])**2 + (rgb[:,2])**2)
bins = np.histogram(rgb_distances, bins=1000)
background = bins[1][np.argmax(bins[0])]

print(xyz.shape)

xyz_planeless = []
rgb_planeless = []
# plane removal
for i, (xyz_, rgb_) in enumerate(zip(xyz, rgb)):
    if abs(background - rgb_dist(rgb_)) < 2:
        if abs(xyz_[2] - mean[2]) > 10**-2:
            xyz_planeless.append(xyz_)
            rgb_planeless.append(rgb_)
            #xyz[i] = np.array([0, 0, 0])
        # if xyz_[0] < -0.1:
        #     xyz[i] = np.array([0, 0, 0])
    #xyz[i] = np.array(xyz_)
print(np.array(xyz_planeless).shape)
# # single object detection
objects=[]
object_reference_points = []
point_colors = []
palette = [[255,0,0],[0,255,0],[0,0,255], [255, 192, 203]]
color_counter = 0
for i, (xyz_, rgb_) in enumerate(zip(xyz, rgb)):
    if (xyz_ != np.array([0,0,0])).any():
        # create first object
        if len(object_reference_points) < 1:
            object_reference_points.append(xyz_)
            objects.append([xyz_])
            point_colors.append(palette[0])
            rgb[i] = np.array(point_colors[0])
            continue
        # go through all objects and check if it is close to refrerence
        was_part_of_object = False
        for j, (ref_point, obj, color) in enumerate(zip(object_reference_points, objects, point_colors)):
            if np.linalg.norm(ref_point - xyz_) < 2*10**-1:
                objects[j].append(xyz_)
                rgb[i] = np.array(color)
                was_part_of_object = True
        # if not part of an object create new object with new color and new reference
        if was_part_of_object == False:
            color_counter = (color_counter + 1) % 3
            object_reference_points.append(xyz_)
            objects.append([xyz_])
            point_colors.append(palette[color_counter])
            rgb[i] = np.array(palette[color_counter])
print(len(objects))
for obj in objects:
    print(len(obj))





xyz_planeless = np.array(xyz_planeless)
rgb_planeless = np.array(rgb_planeless)


ax = plt.axes(projection='3d')
# for obj in objects:
#     plt_primitive_bounding_box(ax, np.array(obj))
ax.set_box_aspect((np.ptp(xyz_planeless[:,0]),np.ptp(xyz_planeless[:,1]), np.ptp(xyz_planeless[:,2])))
ax.scatter(xyz_planeless[:,0], xyz_planeless[:,1], xyz_planeless[:,2],c = rgb_planeless/255 , s=0.01) #c = rgb/255
add_image_bounding_pixels(ax, min_point, max_point)
plt.show()
