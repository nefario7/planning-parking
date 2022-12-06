import numpy as np
from numpy import savetxt
import matplotlib.pyplot as plt
import matplotlib.transforms as mtransforms
import matplotlib.cbook as cbook
import matplotlib.patches as patches
import matplotlib as mpl
import sys

grid_size = 0.2

# FILE_NAME = "b_level"
FILE_NAME = "mit_base"

def plot_path(path_to_waypoints):
    map_arr = np.loadtxt(f"../maps/{FILE_NAME}_map.csv",
                    delimiter=",", dtype=int)

    way_points = np.loadtxt(path_to_waypoints, delimiter=",", dtype=float)
    coords = way_points[:,1:3]
    thetas = way_points[:,-1]
    thetas = np.deg2rad(thetas)
    car_width = 6
    car_length = 8

    x_vector = 40.0 * np.cos(thetas)
    y_vector = 40.0 * np.sin(thetas)

    x_rotated = x_vector * np.cos(-np.pi / 2) - y_vector * np.sin(-np.pi / 2)
    y_rotated = x_vector * np.sin(-np.pi / 2) + y_vector * np.cos(-np.pi / 2)


    plt.imshow(map_arr)
    plt.plot(coords[:,1], coords[:,0], color ='r', linewidth = 1, zorder = 1 )
    
    plt.scatter(coords[:,1], coords[:,0], color ='w', s=1, zorder= 2)
    plt.quiver(coords[:,1], coords[:,0], x_rotated, y_rotated, angles='uv', zorder = 3)
    
    plt.show()


if __name__ == '__main__':
    n = len(sys.argv)
    path_to_waypoints = sys.argv[1]
    plot_path(path_to_waypoints)