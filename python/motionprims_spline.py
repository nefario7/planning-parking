# Generate motion primitives for our robot

import numpy as np
import matplotlib.pyplot as plt
from scipy.interpolate import CubicSpline, CubicHermiteSpline
import json

# Parameters
DIST = 1.6
DELTA_ANGLE = 11.25
GRID_SIZE = 0.2
RESOLUTION = 0.01
NUM_PRIMS = 5
ADDITIONAL_PRIMS = [2, 4]   
PI = 3.141589


def get_y(x, c_arr):

    y = c_arr[0]*(x**3) + c_arr[1]*(x**2) + c_arr[2]*(x**1) + c_arr[3]*(x**0)
    slope = 3*c_arr[0]*(x**2) + 2*c_arr[1]*(x**1) + 1*c_arr[2]*(x**0)
    theta = np.arctan(slope)
    # print("yy-->", y[0])

    return y[0], theta[0]

def create_spline(x_start, y_start, theta_start, x_end, y_end, theta_end):

    x = [x_start, x_end]
    y = [y_start, y_end]
    dydx = [np.tan(theta_start), np.tan(theta_end)]

    spline = CubicHermiteSpline(x, y, dydx)
    xx = list(np.linspace(spline.x[0], spline.x[1], 60))
    yy = []
    theta = []

    for i in xx:
        y_, theta_ = get_y(i, spline.c)
        # yy.append(get_y(i, spline.c))
        yy.append(y_)
        theta.append(theta_)

    # print(len(xx))
    # print(len(yy))

    # print("X-->",xx)
    # print("Y-->",yy)

    return xx, yy, theta

def my_cubic_spline_creator(x1, y1, theta1, x2, y2, theta2):
    X = np.array([[x1**3, x1**2, x1, 1],
                    [x2**3, x2**2, x2, 1],
                    [3*x1**2, 2*x1, 1, 0],
                    [3*x2**2, 2*x2, 1, 0]])
    Y = np.array([[y1], 
                    [y2],
                    [np.tan(theta1)],
                    [np.tan(theta2)]])

    coeff = np.linalg.pinv(X)@Y
    # print(X)
    # print(Y)
    # print(coeff)
    xx = list(np.linspace(x1, x2))
    yy = []
    theta = []
    for i in xx:
        y_, theta_ = get_y(i, coeff)
        # yy.append(get_y(i, spline.c))
        yy.append(y_)
        theta.append(theta_)

    return xx, yy, theta

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


        x_end = int((path_start_x[-1] + GRID_SIZE / 2) / GRID_SIZE)
        y_end = int((path_start_y[-1] + GRID_SIZE / 2) / GRID_SIZE)
        if path_start_x[-1] + GRID_SIZE / 2 < 0:
            x_end -= 1
        if path_start_y[-1] + GRID_SIZE / 2 < 0:
            y_end -= 1

        x_end_pt = x_end * GRID_SIZE
        y_end_pt = y_end * GRID_SIZE
        

        data[idx] = dict()
        data[idx]['start'] = [0, 0, theta*(PI/180)]
        data[idx]['end'] = [x_end_pt, y_end_pt, (theta + alpha)*(PI/180)]
        xx, yy, theta1 = my_cubic_spline_creator(0,0,theta*(PI/180), x_end_pt, y_end_pt, (theta+alpha)*(PI/180))
        # if (theta*(PI/180) == (theta+alpha)*(PI/180)):
        #     xx = list(np.linspace(0, x_end_pt, 60))
        #     yy = list(np.linspace(0, y_end_pt, 60))
        #     theta1=(np.ones(np.linspace(0, x_end_pt, 60).shape)*theta*(PI/180)).tolist()
        # print(len(xx), len(yy), len(path_start_theta.tolist()))
        if np.max(abs(np.asarray(xx))) > 20 or np.max(abs(np.asarray(yy))>20):
            print("--> Ignored: ",0, 0, theta, "<==>", x_end_pt, y_end_pt, (theta + alpha), "<==>", np.max(abs(np.asarray(xx))), np.max(abs(np.asarray(yy)))   )
            continue
        mprims = [xx,yy, theta1]
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


        # x_end = int((path_start_x[-1] - GRID_SIZE / 2) / GRID_SIZE)
        # y_end = int((path_start_y[-1] - GRID_SIZE / 2) / GRID_SIZE)

        x_end = int((path_start_x[-1] + GRID_SIZE / 2) / GRID_SIZE)
        y_end = int((path_start_y[-1] + GRID_SIZE / 2) / GRID_SIZE)
        if path_start_x[-1] + GRID_SIZE / 2 < 0:
            x_end -= 1
        if path_start_y[-1] + GRID_SIZE / 2 < 0:
            y_end -= 1

        x_end_pt = x_end * GRID_SIZE
        y_end_pt = y_end * GRID_SIZE

        data[idx] = dict()
        data[idx]['start'] = [0, 0, theta*(PI/180)]
        data[idx]['end'] = [x_end_pt, y_end_pt, (theta + alpha)*(PI/180)]
        xx, yy, theta1 = my_cubic_spline_creator(0,0,theta*(PI/180), x_end_pt, y_end_pt, (theta+alpha)*(PI/180))
        if np.max(abs(np.asarray(xx))) > 10 or np.max(abs(np.asarray(yy))>10):
            print("==> Ignored")
            continue
        # if (theta*(PI/180) == (theta+alpha)*(PI/180)):
        #     xx = list(np.linspace(0, x_end_pt, 60))
        #     yy = list(np.linspace(0, y_end_pt, 60))
        #     theta1=(np.ones(np.linspace(0, x_end_pt, 60).shape)*theta*(PI/180)).tolist()
        # print(len(xx), len(yy), len(path_start_theta.tolist()))
        mprims = [xx,yy, theta1]
        data[idx]['mprim'] = mprims
        data[idx]['collisions'] = []

        idx += 1
    # print("data: \n", len(data))
    return data

def generate_mprims():
    # Generate motion primitives for all angles
    robot_angles = np.arange(0, 360, DELTA_ANGLE/8)
    # robot_angles = np.arange(0, 360, 10)
    all_mprims = {}
    for theta in robot_angles:
        data = generate_mprims_angle(theta)
        all_mprims[theta] = data
    return all_mprims

# def generate_mprims():
#     # Generate motion primitives for all angles
#     # robot_angles = np.arange(0, 11.25, DELTA_ANGLE)
#     all_mprims = {}
#     data = generate_mprims_angle(0)
#     data2 = data
#     # print("data: \n", data[0]['end'])
#     # print("data: \n", data[1]['end'])
#     # print(len(data))


#     # x_rotated = x_vector * np.cos(-np.pi / 2) - y_vector * np.sin(-np.pi / 2)
#     # y_rotated = x_vector * np.sin(-np.pi / 2) + y_vector * np.cos(-np.pi / 2)
#     # x = np.asarray(data[0]['mprim'][0])
#     # y = np.asarray(data[0]['mprim'][1])
#     # x_start = data[0]['start'][0]
#     # y_start = data[0]['start'][1]
#     # theta_start = data[0]['start'][2]
#     # x_end = data[0]['end'][0]
#     # y_end = data[0]['end'][1]
#     # theta_end = data[0]['end'][2]
#     for angle in np.arange(0, 360, DELTA_ANGLE):
#         for i in np.arange(len(data)):
#             # data2 = data
#             x = np.asarray(data[i]['mprim'][0])
#             y = np.asarray(data[i]['mprim'][1])
#             x_start = data[i]['start'][0]
#             y_start = data[i]['start'][1]
#             theta_start = data[i]['start'][2]
#             x_end = data[i]['end'][0]
#             y_end = data[i]['end'][1]
#             theta_end = data[i]['end'][2]
        

#             print(angle)

#             x_rotated = x * np.cos(angle*(PI/180)) - y*np.sin(angle*(PI/180))
#             y_rotated = x * np.sin(angle*(PI/180)) + y*np.cos(angle*(PI/180))

#             x_end_pt = x_end * np.cos(angle*(PI/180)) - y_end*np.sin(angle*(PI/180))
#             y_end_pt = x_end * np.sin(angle*(PI/180)) + y_end*np.cos(angle*(PI/180))

#             # print("start: ", x_start, y_start)
#             # print("end: ", x_end, y_end, "-->", x_end_pt, y_end_pt)



#             mprim = [x_rotated.tolist(), y_rotated.tolist()]
#             data2[i]['start'] = [x_start, y_start, theta_start+angle]
#             data2[i]['end'] = [x_end_pt, y_end_pt, (theta_end+angle)]
#             data2[i]['mprim'] = mprim

#         all_mprims[angle] = data2
#             # all_mprims[0] = data[0]['end']

#     print(all_mprims)

#     # for theta in robot_angles:
#     #     data = generate_mprims_angle(theta)
#     #     all_mprims1[theta] = data
#     #     print(all[])
#     # print(len(all_mprims1))
#     # print(all_mprims1[theta])    
#     # for key in all_mprims1:
#         # print(key)

#     # for rot_angle in np.arange(45, 360, 11.25):
#     #     for key in all_mprims1:
#     #         print(all_mprims1[key][2])
        
#     return all_mprims

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
    # print("all_mprims:\n", all_mprims)

    # Save the motion primitives
    with open('mprims_spline.json', 'w') as fp:
        json.dump(all_mprims, fp)

    # Save the configuration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
    with open(('mprims_config_spline.json'), 'w') as fp:
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
    # plt.legend()
    plt.show()
        
