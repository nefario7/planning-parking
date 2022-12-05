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


def generate_mprims():
    # Generate motion primitives for all angles
    all_mprims = {}
   
    
    start_x = 0.0  # [m]
    start_y = 0.0  # [m]

    theta_vals_variation = np.array([-45.0,-22.5,0.0,0.0,0.0,22.5,45.0]) #degrees
    min_radius_vals = np.array([1.6,3.2,1.6,1.6,1.6,3.2,1.6])

    # VALUES FOR 0 AS THETA START
    x_vals_1 = np.array([7.0,8.0,8.0,16.0,24.0,8.0,7.0])
    y_vals_1 = np.array([-3.0,-2.0,0.0,0.0,0.0,2.0,3.0])
    theta_vals_1 = 0.0 + theta_vals_variation

    # VALUES FOR 22.5 AS THETA START
    # x_vals_2 = [4.0,30.0,10.0,9.0,8.0,6.0]
    # y_vals_2 = [2.0,15.0,3.0,1.0,6.0,7.0]
    # theta_vals_2 = [22.5,22.5,0,-22.5,45,67.5]
    x_vals_2 = [9.0,10.0,6.0,12.0,24.0,8.0,5.0]
    y_vals_2 = [1.0, 3.0,3.0, 6.0,12.0,6.0,6.0]
    theta_vals_2 = 22.5+theta_vals_variation


    # # #values for 45 as THETA START
    # x_vals_3 = [2.0,24.0,6.0,7.0,4.0,3.0]
    # y_vals_3 = [2.0,24.0,4.0,3.0,6.0,7.0]
    # theta_vals_3 = [45.0,45.0,22.5,0,67.5,90]
    x_vals_3 = [7.0,6.0,5.0,10.0,20.0,4.0,3.0]
    y_vals_3 = [3.0,4.0,5.0,10.0,20.0,6.0,7.0]
    theta_vals_3 = 45 + theta_vals_variation

    #  VALUES FOR 67.5 AS THETA START
    # x_vals_4 =     [2.0 ,15.0, 3.0,  1.0,6.0,7.0]
    # y_vals_4 =     [4.0 ,30.0,10.0,  9.0,8.0,6.0]
    # theta_vals_4 = [67.5,67.5,  90,112.5,45 ,22.5]
    x_vals_4 = [6.0,6.0,12.0,6.0,3.0,3.0,1.0]
    y_vals_4 = [5.0,8.0,24.0,12.0,6.0,10.0,9.0]
    theta_vals_4 = 67.5+theta_vals_variation
    # theta_vals_4 = [ 22.5,45.0,  67.5,  67.5,  67.5,  90.0,  112.5]
    # print(theta_vals_4
    x_vals = np.array([x_vals_1,x_vals_2,x_vals_3,x_vals_4])
    y_vals = np.array([y_vals_1,y_vals_2, y_vals_3,y_vals_4])#,y_vals_3,y_vals_4])
    theta_vals = np.array([theta_vals_1, theta_vals_2,theta_vals_3, theta_vals_4])#,theta_vals_3,theta_vals_4])

    start_angles = [0.0,22.5,45.0,67.5]
    

    # backward_angles_relative = [-pi/8,0.0,pi/8]
    for k in range(0,1): #Rotation matrix to replicate primitives in first quadrant across all quadrants
        rot_angle = (pi/2)*k
        Rot_mat = np.array([[cos(rot_angle),-sin(rot_angle)],[sin(rot_angle),cos(rot_angle)]])
        # for j in range(0,len(start_angles)): #Accounts for all start angles in given quadrant 
        for j in range(0,1): #Accounts for all start angles in given quadrant 
            data = {}
            start_yaw = np.deg2rad(start_angles[j])+rot_angle
            idx = 0
            for i in range(0,len(x_vals[0])): #Accounts for all primitives for given start angle

                [e_x,e_y] = Rot_mat@np.array([x_vals[j][i]*0.2,y_vals[j][i]*0.2])

                e_theta = np.deg2rad(theta_vals[j][i])+rot_angle
               
                rad = min_radius_vals[i]
                [path_x, path_y, path_yaw, length] = generate_dubin_points([start_x, start_y, start_yaw], [e_x, e_y, e_theta], rad, 1/rad,0.01)

                mprims = [path_x.tolist(), path_y.tolist(), path_yaw.tolist()]
              
                x_grid = int(round(e_x/GRID_SIZE,2))
                y_grid = int(round(e_y/GRID_SIZE,2))
                data[idx] = dict()
                # print([start_x, start_y, wrap_to_360(np.rad2deg(start_yaw))])
                print("Angle is :",wrap_to_360(np.rad2deg(e_theta)))
                print(length)
                data[idx]['start'] = [start_x, start_y, wrap_to_360(np.rad2deg(start_yaw))]
                data[idx]['end'] = [x_grid, y_grid, wrap_to_360(np.rad2deg(e_theta))]
                data[idx]['mprim'] = mprims
                data[idx]['collisions'] = []
                idx+=1
                # for l in range(0,len(backward_angles_relative)):
                #     # print(np.rad2deg(e_theta-start_yaw), backward_angles_relative[l])
                #     if(round(e_theta-start_yaw-backward_angles_relative[l],1)==0):
                #         Rot_mat_2 = np.array([[cos(pi),-sin(pi)],[sin(pi),cos(pi)]])
                #         [e_x_2,e_y_2] = Rot_mat_2@np.array([e_x,e_y])
                #         [path_x, path_y, path_yaw, length] = generate_dubin_points([start_x, start_y, pi+start_yaw], [e_x_2, e_y_2, pi+e_theta], rad, 1/rad,0.01)
                #         # print("Start and end are : ")
                #         mprims = [path_x.tolist(), path_y.tolist(), path_yaw.tolist()]
                #         # plt.scatter(curr_x, curr_y)
                #         # plot_arrow(start_x, start_y, start_yaw)
                #         # plot_arrow(e_x, e_y, e_theta)
                #         print(idx)
                #         data[idx] = dict()
                #         data[idx]['start'] = [start_x, start_y, round(np.rad2deg(start_yaw),2)]
                #         data[idx]['end'] = [e_x_2, e_y_2, wrap_to_360(round(np.rad2deg(e_theta),2))]
                #         data[idx]['mprim'] = mprims
                #         data[idx]['collisions'] = []
                #         idx+=1
            
            for i in [1,2,5]:
                [e_x,e_y] = Rot_mat@np.array([x_vals[j][i]*0.2,y_vals[j][i]*0.2])
                e_theta = np.deg2rad(theta_vals[j][i])+rot_angle

                Rot_mat_2 = np.array([[cos(pi),-sin(pi)],[sin(pi),cos(pi)]])
                [e_x_2,e_y_2] = Rot_mat_2@np.array([e_x,e_y])
                e_theta_2 = np.deg2rad(theta_vals[j][i])+rot_angle + pi
                rad = min_radius_vals[i]
                [path_x, path_y, path_yaw, length] = generate_dubin_points([start_x, start_y, pi+start_yaw], [e_x_2, e_y_2, e_theta_2], rad, 1/rad,0.01)
                
                mprims = [path_x.tolist(), path_y.tolist(), path_yaw.tolist()]
              
                x_grid_2 = int(round(e_x_2/GRID_SIZE,2))
                y_grid_2 = int(round(e_y_2/GRID_SIZE,2))
                data[idx] = dict()
                # print([start_x, start_y, wrap_to_360(np.rad2deg(start_yaw))])
                # print([x_grid_2, y_grid_2, wrap_to_360(np.rad2deg(e_theta_2))])
                data[idx]['start'] = [start_x, start_y, wrap_to_360(np.rad2deg(start_yaw))]
                data[idx]['end'] = [x_grid_2, y_grid_2, wrap_to_360(np.rad2deg(e_theta))]
                data[idx]['mprim'] = mprims
                data[idx]['collisions'] = []
                idx+=1

            
            # print(round(np.rad2deg(start_yaw),2))
            all_mprims[wrap_to_360(round(np.rad2deg(start_yaw),2))] = data

            

            

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
        
