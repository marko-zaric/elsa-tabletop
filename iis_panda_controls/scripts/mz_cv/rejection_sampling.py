from scipy.spatial import Delaunay
from scipy.spatial import ConvexHull
import numpy as np

def uniform_convex_hull_sample(points, N):
    verts = points[ConvexHull(points).vertices]
    delauney_hull = Delaunay(verts)

    X_max = max(verts[:,0])
    X_min = min(verts[:,0])
    Y_max = max(verts[:,1])
    Y_min = min(verts[:,1])
    
    
    sampled_points = []
    num_points_sampled = 0
    while num_points_sampled < N:
        X = np.random.uniform(low = X_min, high = X_max)
        Y = np.random.uniform(low = Y_min, high = Y_max)
        if delauney_hull.find_simplex([X, Y]) >= 0:
            sampled_points.append([X, Y])
            num_points_sampled += 1

    return np.array(sampled_points)