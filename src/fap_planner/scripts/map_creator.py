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

def create_csv():
    points = np.loadtxt("../maps/" + FILE_NAME + "_3d.csv", delimiter=",")
    row_idx = np.asarray(np.where(np.logical_and(points[:,2]>0.3, points[:,2]<2.2)))
    scaled_points = points[row_idx[0,:], 0:4]
    scaled_points = np.around(points / grid_size, decimals=0).astype(int)
    
    # only for x,y columns
    min_x = np.min(scaled_points[:,0])
    min_y = np.min(scaled_points[:,1])

    scaled_points[:,0] = scaled_points[:,0] - np.min(scaled_points[:,0])
    scaled_points[:,1] = scaled_points[:,1] - np.min(scaled_points[:,1])
    
    max_x = np.max(scaled_points[:,0])
    max_y = np.max(scaled_points[:,1])

    print("min x, min y, max x, max y : ", min_x, min_y, max_x, max_y)

    base_map = np.ones((max_x, max_y), dtype=int) * -1

    for i in range(scaled_points.shape[0]):
        z = scaled_points[i,2]
        if z > 3 and z < 8:
            base_map[scaled_points[i,0]-1, scaled_points[i,1]-1] = 1
        elif z <= 3:
            base_map[scaled_points[i,0]-1, scaled_points[i,1]-1] = 0
    
 
    print("base map shape: ",base_map.shape)
    
    file_name = f"../maps/{FILE_NAME}_map.csv"
    savetxt(file_name, base_map, delimiter=",", fmt='%d')
    print("Saved base map!!")
    
    plt.imshow(base_map)
    plt.savefig(f"../maps/{FILE_NAME}_map.png")
    # plt.show()

    # print(base_map.shape)


if __name__ == '__main__':
    create_csv()