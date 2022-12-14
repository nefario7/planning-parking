# Generate motion primitives for our robot

import numpy as np
import matplotlib.pyplot as plt
import json

# Parameters
DIST = 0.6
DELTA_ANGLE = 11.25
GRID_SIZE = 0.2
RESOLUTION = 0.01
NUM_PRIMS = 5
ADDITIONAL_PRIMS = [2, 4]   

# Generate motion primitives
print("Generating Motion Primitives...")

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
        mprims = [path_start_x.tolist(), path_start_y.tolist(), path_start_theta.tolist()]


        x_end = int((path_start_x[-1] - GRID_SIZE / 2) / GRID_SIZE)
        y_end = int((path_start_y[-1] - GRID_SIZE / 2) / GRID_SIZE)

        data[idx] = dict()
        data[idx]['start'] = [0, 0, theta]
        data[idx]['end'] = [x_end, y_end, theta + alpha]
        data[idx]['mprim'] = mprims
        data[idx]['collisions'] = []

        idx += 1
        
    for i in ADDITIONAL_PRIMS:
        alpha = 0
        long_DIST = i * DIST

        path_start_r = np.arange(0, long_DIST, fineness)
        num_points = int(long_DIST / fineness)

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
        mprims = [path_start_x.tolist(), path_start_y.tolist(), path_start_theta.tolist()]


        x_end = int((path_start_x[-1] - GRID_SIZE / 2) / GRID_SIZE)
        y_end = int((path_start_y[-1] - GRID_SIZE / 2) / GRID_SIZE)

        data[idx] = dict()
        data[idx]['start'] = [0, 0, theta]
        data[idx]['end'] = [x_end, y_end, theta + alpha]
        data[idx]['mprim'] = mprims
        data[idx]['collisions'] = []

        idx += 1

    return data

def generate_mprims():
    # Generate motion primitives for all angles
    robot_angles = np.arange(0, 11.25, DELTA_ANGLE)
    all_mprims = {}
    for theta in robot_angles:
        data = generate_mprims_angle(theta)
        all_mprims[theta] = data
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
    with open('mprims.json', 'w') as fp:
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

            plt.plot(data1['mprim'][0], data1['mprim'][1], label = "Angle: " + str(angle) + " Motion Primitive: " + str(idx))

    plt.show()
        
