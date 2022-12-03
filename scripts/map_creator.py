import numpy as np
from numpy import savetxt
import matplotlib.pyplot as plt

grid_size = 0.2

def create_viz():
    points = np.loadtxt("xyz_raw.csv", delimiter=",")
    print(points.shape)
    plt.scatter(points[:,0], points[:,1], c ="blue", s = 2)
    plt.show()

def create_csv():
    points = np.loadtxt("../maps/xyz_raw_3d.csv", delimiter=",")
    row_idx = np.asarray(np.where(np.logical_and(points[:,2]>0.3, points[:,2]<2.2)))
    scaled_points = points[row_idx[0,:], 0:4]
    scaled_points = np.around(points / grid_size, decimals=0).astype(int)
    
    # only for x,y columns
    scaled_points[:,0] = scaled_points[:,0] - np.min(scaled_points[:,0])
    scaled_points[:,1] = scaled_points[:,1] - np.min(scaled_points[:,1])
    min_x = np.min(scaled_points[:,0])
    min_y = np.min(scaled_points[:,1])
    max_x = np.max(scaled_points[:,0])
    max_y = np.max(scaled_points[:,1])
    # print(min_x, max_x, min_y, max_y)
    # plt.scatter(scaled_points[:,0], scaled_points[:,1], c ="blue", s = 2)
    # plt.show()
    # base_map = np.zeros((max_x, max_y), dtype=int)
    base_map = np.ones((max_x, max_y), dtype=int) * -1
    # print(base_map.shape, "->", scaled_points.shape)

    for i in range(scaled_points.shape[0]):
        # base_map[scaled_points[i,0]-1, scaled_points[i,1]-1] = 2
        z = scaled_points[i,2]
        if z > 3:
            base_map[scaled_points[i,0]-1, scaled_points[i,1]-1] = 1
        else:
            base_map[scaled_points[i,0]-1, scaled_points[i,1]-1] = 0


        # base_map[scaled_points[i,0]-1, scaled_points[i,1]-1] = scaled_points[i,2]

    # base_map = base_map - 1
    
 
    print("base map shape: ",base_map.shape)
    
    file_name = "../maps/base_map.csv"
    savetxt(file_name, base_map, delimiter=",", fmt='%d')
    print("Saved base map!!")
    
    plt.imshow(base_map)
    plt.savefig("../maps/base_map.png")
    # plt.show()

    # print(base_map.shape)

def rough():
    points = np.loadtxt("xyz_raw_3d.csv", delimiter=",")
    min_x = np.min(points[:,0])
    min_y = np.min(points[:,1])
    max_x = np.max(points[:,0])
    max_y = np.max(points[:,1])
    min_z = np.min(points[:,2])
    max_z = np.max(points[:,2])
    print(min_z, max_z)
    





if __name__ == '__main__':
    create_csv()
    # create_viz()
    # rough()