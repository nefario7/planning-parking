import numpy as np
from numpy import savetxt
import matplotlib.pyplot as plt
import matplotlib.transforms as mtransforms
import matplotlib.cbook as cbook
import matplotlib.patches as patches
import matplotlib as mpl

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

def get_image():
    fn = cbook.get_sample_data("necked_tensile_specimen.png")
    arr = plt.imread(fn)
    # make background transparent
    # you won't have to do this if your car image already has a transparent background
    mask = (arr == (1,1,1,1)).all(axis=-1)
    arr[mask] = 0
    return arr

def plot_robot(fig, coords, thetas):
    for i in range(coords.shape[0]):
        # ax = fig.add_subplot(111)

        position = (coords[i][0], coords[i][1])

        r1 = patches.Rectangle(position, 3, 4, color="blue", alpha=0.50)
        t2 = mpl.transforms.Affine2D().rotate_deg(thetas[i])#  + ax.transData
        r1.set_transform(t2)

        fig.add_patch(r1)
    

def plot_path():
    map_arr = np.loadtxt("mit_base_map.csv",
                    delimiter=",", dtype=int)

    way_points = np.loadtxt("mit_base_map_wp_l_turn.txt", delimiter=",", dtype=float)
    coords = way_points[:,1:3]
    thetas = way_points[:,-1]
    thetas = np.deg2rad(thetas)
    car_width = 6
    car_length = 8

    x_vector = 40.0 * np.cos(thetas)
    y_vector = 40.0 * np.sin(thetas)

    # x_direction = np.cos(np.deg2rad(way_points[:,2]))
    # y_direction = np.sin(np.deg2rad(way_points[:,2]))

    x_rotated = x_vector * np.cos(-np.pi / 2) - y_vector * np.sin(-np.pi / 2)
    y_rotated = x_vector * np.sin(-np.pi / 2) + y_vector * np.cos(-np.pi / 2)
    # print(coords.shape)

    # fig = plt.figure()

    # plot_robot(fig, coords, thetas)

    plt.imshow(map_arr)
    plt.plot(coords[:,1], coords[:,0], color ='r', linewidth = 1, zorder = 0)
    
    # plt.quiver(coords[:,1], coords[:,0], thetas, angles='uv')
    plt.scatter(coords[:,1], coords[:,0], color ='b', s=1, zorder= 1)
    plt.quiver(coords[:,1], coords[:,0], x_rotated, y_rotated, angles='uv', zorder = 2)
    
    plt.show()


if __name__ == '__main__':
    # create_csv()
    # create_viz()
    # rough()
    plot_path()