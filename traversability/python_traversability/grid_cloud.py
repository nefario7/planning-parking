from turtle import hideturtle
import numpy as np
from python_traversability.config import RESOLUTION, CROP_HEIGHT, VIS
import pyvista as pv
from tqdm import tqdm


def grid_cloud(
    points, is_ground, resolution=RESOLUTION, crop_height=CROP_HEIGHT, vis=VIS
):
    mins = np.min(points, axis=0)
    maxes = np.max(points, axis=0)
    x_segments = np.arange(mins[0], maxes[0], resolution)
    y_segments = np.arange(mins[1], maxes[1], resolution)
    edges = [x_segments, y_segments]
    coords = [np.digitize(points.T[i], b, right=True) for i, b in enumerate(edges)]
    normalized_coords = [(x % 6) / 5 for x in coords]
    colors = np.vstack(normalized_coords + [np.zeros_like(normalized_coords[0])]).T
    if vis:
        plotter = pv.Plotter()
        plotter.add_mesh(pv.PolyData(points), scalars=colors, rgb=True)
        plotter.show()

    filtered_points = []
    filtered_is_ground = []

    # Bottleneck
    for i in tqdm(range(len(x_segments))):
        for j in range(len(y_segments)):
            subset_inds = np.logical_and(coords[0] == i, coords[1] == j)
            subset_points = points[subset_inds]
            subset_is_ground = is_ground[subset_inds]

            ground_subset_points = subset_points[subset_is_ground]

            if len(ground_subset_points) > 0:
                if subset_is_ground.dtype is not np.dtype("bool"):
                    raise ValueError()
                highest_ground = np.max(ground_subset_points[:, 2])
                height_above = subset_points[:, 2] - highest_ground
                valid_height_above = height_above < crop_height

                valid_points = subset_points[valid_height_above]
                valid_is_ground = subset_is_ground[valid_height_above]

                filtered_points.append(valid_points)
                filtered_is_ground.append(valid_is_ground)
    filtered_points = np.vstack(filtered_points)
    filtered_is_ground = np.hstack(filtered_is_ground)
    return filtered_points, filtered_is_ground
