import numpy as np
from scipy.interpolate import CubicSpline, CubicHermiteSpline
import matplotlib.pyplot as plt

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
    xx = list(np.linspace(spline.x[0], spline.x[1]))
    yy = []
    theta = []

    for i in xx:
        y_, theta_ = get_y(i, spline.c)
        # yy.append(get_y(i, spline.c))
        yy.append(y_)
        theta.append(theta_)

    # print(len(xx))
    # print(len(yy))
    print(len(theta))

    # print("X-->",xx)
    # print("Y-->",yy)
    print("T-->", theta)

    return xx, yy, theta






if __name__=='__main__':

    # x_start = 0
    # y_start = 0
    # theta_start = 0

    # x_end = 2
    # y_end = 2
    # delta_theta = PI/8
    # for i in range(3):
    #     xx, yy = create_spline(x_start, y_start, theta_start, x_end, y_end, theta_start + (delta_theta*i))
    #     print("End: ", x_end, y_end, (180/PI) * (theta_start + (delta_theta*i)))
    #     plt.plot(xx, yy)

    #     xx, yy = create_spline(x_start, y_start, theta_start, x_end, -y_end, theta_start - (delta_theta*i))
    #     print("End: ", x_end, y_end, (180/PI) * (theta_start - (delta_theta*i)))
    #     plt.plot(xx, yy)

    # xx, yy = create_spline(x_start, y_start, theta_start, x_end, 0, 0)
    # print("End: ", x_end, 0, (180/PI) * (0))
    # plt.plot(xx, yy)

    # plt.show()

    xx1, yy1, theta1 = create_spline(0,0, 0, 2,1, PI/8)
    xx2, yy2, theta2 = create_spline(0,0, 0, 2,-1, -PI/8)
    xx3, yy3, theta3 = create_spline(0,0, 0, 2,2, PI/4)
    xx4, yy4, theta4 = create_spline(0,0, 0, 2,-2, -PI/4)
    xx5, yy5, theta5 = create_spline(0,0, 0, 2,0, 0)
    xx6, yy6, theta6 = create_spline(0,0, 0, 2,2, PI/3)
    xx7, yy7, theta7 = create_spline(0,0, 0, 2,-2, -PI/3)
    plt.plot(xx1, yy1)
    plt.plot(xx2, yy2)
    plt.plot(xx3, yy3)
    plt.plot(xx4, yy4)
    plt.plot(xx5, yy5)
    plt.plot(xx6, yy6)
    plt.plot(xx7, yy7)

    # plt.show()