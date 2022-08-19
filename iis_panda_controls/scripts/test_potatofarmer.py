from os.path import join
from scipy.spatial.transform import Rotation as R
import numpy as np
import random
from itertools import product


vec = np.array([1,-1, 1])

vec = vec / np.linalg.norm(vec)


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

print(azimuth*360/(2*np.pi))






exit()

r = R.from_euler('xyz', [0, 0, np.pi/2])

print(r.as_quat())

print(0.7/0.15)
print(0.5/0.15)

num = -0.3
for i in range(4):
    num += 0.15
    print(num)

num = 0.2
for i in range(3):
    num += 0.15
    print(num)

x_grid = [-0.15, 0, 0.15, 0.3]
y_grid = [0.2, 0.35, 0.5, 0.65]

xy_grid = list(product(x_grid, y_grid))
print(len(xy_grid))
xy_grid.pop(xy_grid.index(random.choice(xy_grid)))

print(len(xy_grid))