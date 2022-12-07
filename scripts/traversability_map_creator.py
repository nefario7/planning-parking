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

def closest_node(free_point, obstacle_points):
    obs_nodes = np.asarray(obstacle_points)
    obs_X = obs_nodes.T[0]
    obs_Y = obs_nodes.T[1]
    # print(np.shape(free_point))
    dist_2 = ((obs_X - free_point[0])**2+(obs_Y - free_point[1])**2)
    # print("Squared distance is:",np.min(dist_2))
    # print(free_point)
    # print([obs_X[np.argmin(dist_2)],obs_Y[np.argmin(dist_2)]])
    return [np.min(dist_2)**0.5,np.argmin(dist_2)]
    # return 0

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

    # print("min x, min y, max x, max y : ", min_x, min_y, max_x, max_y)

    base_map = np.ones((max_x, max_y), dtype=int) * -1

    # obstacle_points = np.array([[]])
    obstacle_points =  np.empty(shape=(0,2))
    # free_points = np.array([[]])
    free_points =  np.empty(shape=(0,2))
    # check_points = np.array([[0,0]])
    for i in range(scaled_points.shape[0]):
    # for i in range(2000):
        z = scaled_points[i,2]
        new_point = np.array([[scaled_points[i,0]-1, scaled_points[i,1]-1]])
        if z > 3 and z < 8:
            base_map[scaled_points[i,0]-1, scaled_points[i,1]-1] = 1
            # obstacle_points = np.append(obstacle_points,new_point,0)
            # print("Adding to obstacles", new_point)
        elif z <= 3:
            base_map[scaled_points[i,0]-1, scaled_points[i,1]-1] = 0
            # free_points = np.append(free_points,new_point,0)
            # print("Adding to free space",new_point)

    for i in range(scaled_points.shape[0]):
        new_point = np.array([[scaled_points[i,0]-1, scaled_points[i,1]-1]])
        if(base_map[scaled_points[i,0]-1, scaled_points[i,1]-1]==1):
            obstacle_points = np.append(obstacle_points,new_point,0)
        elif (base_map[scaled_points[i,0]-1, scaled_points[i,1]-1]==0):
            free_points = np.append(free_points,new_point,0)
    
    park_map = np.ones((max_x, max_y), dtype=int) * -1

    collision_thresh = 0
    print("Starting park check")
    for ele in free_points:
        # print(ele)
        [dist,index] = closest_node(ele,obstacle_points)
        if(dist<collision_thresh):
            park_map[int(ele[0]),int(ele[1])] = -1
        else:
            park_map[int(ele[0]),int(ele[1])]= dist-collision_thresh


    # print(np.shape(check_points))
    print(np.shape(free_points),np.shape(obstacle_points))
    # print("Index is :",dist)
    # print("base map shape: ",base_map.shape)
    
    # file_name = f"../maps/{FILE_NAME}_map.csv"
    # savetxt(file_name, base_map, delimiter=",", fmt='%d')
    # print("Saved base map!!")

    file_name = f"../maps/{FILE_NAME}_traversability_map.csv"
    savetxt(file_name, park_map, delimiter=",", fmt='%d')
    print("Saved park map!!")
    
    # plt.imshow(base_map)
    # plt.savefig(f"../maps/{FILE_NAME}_map.png")
    # plt.show()
    # plt.scatter(free_points.T[0],free_points.T[1],color='blue',s=0.1)
    # plt.scatter(obstacle_points.T[0],obstacle_points.T[1],color='red',s=0.1)
    # plt.show()
    # plt.imshow(park_map)
    # plt.show()
    # print(base_map.shape)


if __name__ == '__main__':
    create_csv()