# Generate motion primitives for our robot

import numpy as np
import matplotlib.pyplot as plt
from numpy import pi
from numpy import cos
from numpy import sin
import json

from dubins_path import generate_dubin_points

# Parameters
DIST = 1.6
START_ANGLE = 0
DELTA_ANGLE = 22.5
END_ANGLE = 360
GRID_SIZE = 0.2
RESOLUTION = 0.01
NUM_PRIMS = 5
ADDITIONAL_PRIMS = [2, 4]   

# Generate motion primitives
print("Generating Motion Primitives...")

def wrap_to_360(angle):
    while angle >= 360:
        angle -= 360
    while angle < 0:
        angle += 360
    return angle

def generate_mprims_angle(theta, num_prims = NUM_PRIMS, fineness = RESOLUTION):
    # Generate motion primitives for a given angle
    max_alpha = (num_prims - 1) * DELTA_ANGLE / 2
    alpha_angles = np.arange(-max_alpha, max_alpha + DELTA_ANGLE, DELTA_ANGLE)

    data = {}
    idx = 0
    for alpha in alpha_angles:
        path_start_r = np.arange(0, DIST, fineness)
        num_points = int(DIST / fineness)

        path_start_alpha_small = np.linspace(0, alpha, num_points*10)
        path_start_theta_small = theta + path_start_alpha_small
        path_start_theta_small = path_start_theta_small * np.pi / 180
        path_start_alpha = np.linspace(0, alpha, num_points)
        path_start_theta = theta + path_start_alpha
        path_start_theta = path_start_theta * np.pi / 180
        path_start_x_small = np.ones(10*num_points)
        path_start_y_small = np.ones(10*num_points)
        path_start_x = np.ones(num_points)
        path_start_y = np.ones(num_points)

        path_start_x[0] = 0
        path_start_y[0] = 0
        path_start_x_small[0] = 0
        path_start_y_small[0] = 0
        for i in range(1, 10*num_points):
            avg_angle = (path_start_theta_small[i] + path_start_theta_small[i-1]) / 2
            path_start_x_small[i] = path_start_x_small[i-1] + fineness / 10 * np.cos(avg_angle)
            path_start_y_small[i] = path_start_y_small[i-1] + fineness / 10 * np.sin(avg_angle)
            if i%10 == 0:
                path_start_x[int(i/10)] = path_start_x_small[i]
                path_start_y[int(i/10)] = path_start_y_small[i]

        # path_start_x = path_start_r * np.cos(path_start_theta)
        # path_start_y = path_start_r * np.sin(path_start_theta)

        x_end = int((path_start_x[-1] + GRID_SIZE / 2) / GRID_SIZE)
        y_end = int((path_start_y[-1] + GRID_SIZE / 2) / GRID_SIZE)
        if path_start_x[-1] + GRID_SIZE / 2 < 0:
            x_end -= 1
        if path_start_y[-1] + GRID_SIZE / 2 < 0:
            y_end -= 1

        x_end_pt = x_end * GRID_SIZE
        y_end_pt = y_end * GRID_SIZE

        theta_rad = np.deg2rad(theta)
        
        theta_plus_alpha_rad = np.deg2rad(theta + alpha)
        # print("new theta is in degs",theta+alpha)

        #! Implement wrap2pi on alpha + theta       - Done
        #! Correct the scaled up straight lines     - Done
#         Start:  [0, 0, 5.8904862254808625]
#         End:  [15, -6, 5.8904862254808625]
#         Start:  [0, 0, 5.8904862254808625]
#         End:  [30, -12, 5.8904862254808625]

        start_point = [0, 0, theta_rad]
        end_point = [x_end_pt, y_end_pt, theta_plus_alpha_rad]
        curvature = 1/1.6
        # /curvature = abs(alpha / DELTA_ANGLE) / 1.6

        # print("Start point for dubins is : ", [start_point[0],start_point[1],np.rad2deg(start_point[2])])
        # print("End point for dubins is : ", [end_point[0],end_point[1],np.rad2deg(end_point[2])])
        [path_x, path_y, path_yaw, length] = generate_dubin_points(start_point, end_point, 1.6, curvature, RESOLUTION)

        # mprims = [path_start_x.tolist(), path_start_y.tolist(), path_start_theta.tolist()]
        mprims = [path_x.tolist(), path_y.tolist(), path_yaw.tolist()]

        data[idx] = dict()
        data[idx]['start'] = [0, 0, theta]
        data[idx]['end'] = [x_end, y_end, wrap_to_360(theta+alpha)]
        data[idx]['mprim'] = mprims
        data[idx]['collisions'] = []

        idx += 1

    return data

def generate_mprims():
    # Generate motion primitives for all angles
    # robot_angles = np.arange(START_ANGLE, END_ANGLE, DELTA_ANGLE)
    all_mprims = {}
    # for theta in robot_angles:
    #     data = generate_mprims_angle(theta)
    #     all_mprims[theta] = data
    
    start_x = 0.0  # [m]
    start_y = 0.0  # [m]
    # start_yaw = np.deg2rad(67.5)  # [rad]
    
    # VALUES FOR 0 AS THETA START
    x_vals_1 = [7.0,8.0,3.0,30.0,8.0,7.0]
    y_vals_1 = [3.0,2.0,0.0,0.0,-2.0,-3.0]
    theta_vals_1 = [45,22.5,0.0,0.0,-22.5,-45]
    min_radius_vals_1 = [1.6,3.2,1.6,1.6,3.2,1.6]

    # VALUES FOR 22.5 AS THETA START
    x_vals_2 = [4.0,30.0,10.0,9.0,8.0,6.0]
    y_vals_2 = [2.0,15.0,3.0,1.0,6.0,7.0]
    theta_vals_2 = [22.5,22.5,0,-22.5,45,67.5]
    min_radius_vals_2 = [1.6,1.6,3.2,1.6,3.2,1.6]

    # # #values for 44.5 as THETA START
    x_vals_3 = [2.0,24.0,6.0,7.0,4.0,3.0]
    y_vals_3 = [2.0,24.0,4.0,3.0,6.0,7.0]
    theta_vals_3 = [45.0,45.0,22.5,0,67.5,90]
    min_radius_vals_3 = [1.6,1.6,3.2,1.6,3.2,1.6]

    #  VALUES FOR 67.5 AS THETA START
    y_vals_4 = [4.0,30.0,10.0,9.0,8.0,6.0]
    x_vals_4 = [2.0,15.0,3.0,1.0,6.0,7.0]
    theta_vals_4 = [67.5,67.5,90,112.5,45,22.5]
    min_radius_vals_4 = [1.6,1.6,3.2,1.6,3.2,1.6]

    x_vals = np.array([x_vals_1,x_vals_2,x_vals_3,x_vals_4])
    y_vals = np.array([y_vals_1,y_vals_2,y_vals_3,y_vals_4])
    theta_vals = np.array([theta_vals_1,theta_vals_2,theta_vals_3,theta_vals_4])
    min_radius_vals = np.array([min_radius_vals_1,min_radius_vals_2,min_radius_vals_3,min_radius_vals_4])

    start_angles = [0.0,22.5,45.0,67.5]

    backward_angles_relative = [-pi/8,0.0,pi/8]
    for k in range(0,4):
        rot_angle = (pi/2)*k
        Rot_mat = np.array([[cos(rot_angle),-sin(rot_angle)],[sin(rot_angle),cos(rot_angle)]])
        for j in range(0,len(start_angles)):
            data = {}
            start_yaw = np.deg2rad(start_angles[j])+rot_angle
            # print([start_x,start_y,start_yaw])
            # print(np.deg2rad(start_yaw))
            idx = 0
            for i in range(0,len(x_vals[0])):
                # print(i)
                # e_x = Rot_mat*(x_vals[j][i]*0.2)
                # e_y = Rot_mat*(y_vals[j][i]*0.2)

                [e_x,e_y] = Rot_mat@np.array([x_vals[j][i]*0.2,y_vals[j][i]*0.2])

                e_theta = np.deg2rad(theta_vals[j][i])+rot_angle
               
                rad = min_radius_vals[j][i]
                [path_x, path_y, path_yaw, length] = generate_dubin_points([start_x, start_y, start_yaw], [e_x, e_y, e_theta], rad, 1/rad,0.01)

                mprims = [path_x.tolist(), path_y.tolist(), path_yaw.tolist()]
                # plt.scatter(curr_x, curr_y)
                # plot_arrow(start_x, start_y, start_yaw)
                # plot_arrow(e_x, e_y, e_theta)
                data[idx] = dict()
                data[idx]['start'] = [start_x, start_y, np.rad2deg(start_yaw)]
                data[idx]['end'] = [e_x, e_y, wrap_to_360(np.rad2deg(e_theta))]
                data[idx]['mprim'] = mprims
                data[idx]['collisions'] = []
                print(idx)
                idx+=1
                for l in range(0,len(backward_angles_relative)):
                    # print(np.rad2deg(e_theta-start_yaw), backward_angles_relative[l])
                    if(round(e_theta-start_yaw-backward_angles_relative[l],1)==0):
                        Rot_mat_2 = np.array([[cos(pi),-sin(pi)],[sin(pi),cos(pi)]])
                        [e_x_2,e_y_2] = Rot_mat_2@np.array([e_x,e_y])
                        [path_x, path_y, path_yaw, length] = generate_dubin_points([start_x, start_y, pi+start_yaw], [e_x_2, e_y_2, pi+e_theta], rad, 1/rad,0.01)
                        # print("Start and end are : ")
                        mprims = [path_x.tolist(), path_y.tolist(), path_yaw.tolist()]
                        # plt.scatter(curr_x, curr_y)
                        # plot_arrow(start_x, start_y, start_yaw)
                        # plot_arrow(e_x, e_y, e_theta)
                        print(idx)
                        data[idx] = dict()
                        data[idx]['start'] = [start_x, start_y, round(np.rad2deg(start_yaw),2)]
                        data[idx]['end'] = [e_x, e_y, wrap_to_360(round(np.rad2deg(e_theta),2))]
                        data[idx]['mprim'] = mprims
                        data[idx]['collisions'] = []
                        idx+=1
            print(round(np.rad2deg(start_yaw),2))
            all_mprims[round(np.rad2deg(start_yaw),2)] = data



            

        # break

    # plt.legend()
    # plt.grid(True)
    # plt.axis("equal")
    # plt.show()

    return all_mprims

def generate_config():
    # Generate configuration file
    config = {}
    config['distance'] = DIST
    config['grid_size'] = GRID_SIZE
    config['delta_angle'] = DELTA_ANGLE
    config['resolution'] = RESOLUTION
    config['num_prims'] = NUM_PRIMS
    config['additional_prims'] = ADDITIONAL_PRIMS

    return config


if __name__ == "__main__":
    all_mprims = generate_mprims()
    config_mprims = generate_config()

    # Save the motion primitives
    with open('mprims_dubin_updated.json', 'w') as fp:
        json.dump(all_mprims, fp)

    # Save the configuration
    with open(('mprims_config.json'), 'w') as fp:
        json.dump(config_mprims, fp)
    
    # Plot the motion primitives
    plt.figure()
    for angle, data in all_mprims.items():
        # print("Angle: ", angle)
        for idx, data1 in data.items():
            # print("Motion Primitive: ", idx, end = " ")
            # print("X: ", mprim[0], end = " ")
            # print("Y: ", mprim[1], end = " ")
            # print("Theta: ", mprim[2])

            # print("Start: ", data1["start"])
            # print("End: ", data1["end"])

            plt.plot(data1['mprim'][0], data1['mprim'][1], label = "Angle: " + str(angle) + " Motion Primitive: " + str(idx))

    plt.show()
        
