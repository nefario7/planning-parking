"""

Dubins path planner sample code

author Atsushi Sakai(@Atsushi_twi)

"""
import sys
import pathlib
# sys.path.append(str(pathlib.Path(__file__).parent.parent.parent))

from math import sin, cos, atan2, sqrt, acos, pi, hypot
import numpy as np
import numpy as np
from numpy import pi
from scipy.spatial.transform import Rotation as Rot


def rot_mat_2d(angle):
    """
    Create 2D rotation matrix from an angle

    Parameters
    ----------
    angle :

    Returns
    -------
    A 2D rotation matrix

    Examples
    --------
    >>> angle_mod(-4.0)


    """
    return Rot.from_euler('z', angle).as_matrix()[0:2, 0:2]


def angle_mod(x, zero_2_2pi=False, degree=False):
    """
    Angle modulo operation
    Default angle modulo range is [-pi, pi)

    Parameters
    ----------
    x : float or array_like
        A angle or an array of angles. This array is flattened for
        the calculation. When an angle is provided, a float angle is returned.
    zero_2_2pi : bool, optional
        Change angle modulo range to [0, 2pi)
        Default is False.
    degree : bool, optional
        If True, then the given angles are assumed to be in degrees.
        Default is False.

    Returns
    -------
    ret : float or ndarray
        an angle or an array of modulated angle.

    Examples
    --------
    >>> angle_mod(-4.0)
    2.28318531

    >>> angle_mod([-4.0])
    np.array(2.28318531)

    >>> angle_mod([-150.0, 190.0, 350], degree=True)
    array([-150., -170.,  -10.])

    >>> angle_mod(-60.0, zero_2_2pi=True, degree=True)
    array([300.])

    """
    if isinstance(x, float):
        is_float = True
    else:
        is_float = False

    x = np.asarray(x).flatten()
    if degree:
        x = np.deg2rad(x)

    if zero_2_2pi:
        mod_angle = x % (2 * np.pi)
    else:
        mod_angle = (x + np.pi) % (2 * np.pi) - np.pi

    if degree:
        mod_angle = np.rad2deg(mod_angle)

    if is_float:
        return mod_angle.item()
    else:
        return mod_angle
show_animation = True


def plan_dubins_path(s_x, s_y, s_yaw, g_x, g_y, g_yaw, curvature,
                     step_size=0.01, selected_types=None):
    """
    Plan dubins path

    Parameters
    ----------
    s_x : float
        x position of the start point [m]
    s_y : float
        y position of the start point [m]
    s_yaw : float
        yaw angle of the start point [rad]
    g_x : float
        x position of the goal point [m]
    g_y : float
        y position of the end point [m]
    g_yaw : float
        yaw angle of the end point [rad]
    curvature : float
        curvature for curve [1/m]
    step_size : float (optional)
        step size between two path points [m]. Default is 0.1
    selected_types : a list of string or None
        selected path planning types. If None, all types are used for
        path planning, and minimum path length result is returned.
        You can select used path plannings types by a string list.
        e.g.: ["RSL", "RSR"]

    Returns
    -------
    x_list: array
        x positions of the path
    y_list: array
        y positions of the path
    yaw_list: array
        yaw angles of the path
    modes: array
        mode list of the path
    lengths: array
        arrow_length list of the path segments.

    Examples
    --------
    You can generate a dubins path.

    >>> start_x = 1.0  # [m]
    >>> start_y = 1.0  # [m]
    >>> start_yaw = np.deg2rad(45.0)  # [rad]
    >>> end_x = -3.0  # [m]
    >>> end_y = -3.0  # [m]
    >>> end_yaw = np.deg2rad(-45.0)  # [rad]
    >>> curvature = 1.0
    >>> path_x, path_y, path_yaw, mode, _ = plan_dubins_path(
                start_x, start_y, start_yaw, end_x, end_y, end_yaw, curvature)
    >>> plt.plot(path_x, path_y, label="final course " + "".join(mode))
    >>> plot_arrow(start_x, start_y, start_yaw)
    >>> plot_arrow(end_x, end_y, end_yaw)
    >>> plt.legend()
    >>> plt.grid(True)
    >>> plt.axis("equal")
    >>> plt.show()

    .. image:: dubins_path.jpg
    """
    if selected_types is None:
        planning_funcs = _PATH_TYPE_MAP.values()
    else:
        planning_funcs = [_PATH_TYPE_MAP[ptype] for ptype in selected_types]

    # calculate local goal x, y, yaw
    l_rot = rot_mat_2d(s_yaw)
    le_xy = np.stack([g_x - s_x, g_y - s_y]).T @ l_rot
    local_goal_x = le_xy[0]
    local_goal_y = le_xy[1]
    local_goal_yaw = g_yaw - s_yaw

    lp_x, lp_y, lp_yaw, modes, lengths = _dubins_path_planning_from_origin(
        local_goal_x, local_goal_y, local_goal_yaw, curvature, step_size,
        planning_funcs)

    # Convert a local coordinate path to the global coordinate
    rot = rot_mat_2d(-s_yaw)
    converted_xy = np.stack([lp_x, lp_y]).T @ rot
    x_list = converted_xy[:, 0] + s_x
    y_list = converted_xy[:, 1] + s_y
    yaw_list = angle_mod(np.array(lp_yaw) + s_yaw)

    return x_list, y_list, yaw_list, modes, lengths


def _mod2pi(theta):
    return angle_mod(theta, zero_2_2pi=True)


def _calc_trig_funcs(alpha, beta):
    sin_a = sin(alpha)
    sin_b = sin(beta)
    cos_a = cos(alpha)
    cos_b = cos(beta)
    cos_ab = cos(alpha - beta)
    return sin_a, sin_b, cos_a, cos_b, cos_ab


def _LSL(alpha, beta, d):
    sin_a, sin_b, cos_a, cos_b, cos_ab = _calc_trig_funcs(alpha, beta)
    mode = ["L", "S", "L"]
    p_squared = 2 + d ** 2 - (2 * cos_ab) + (2 * d * (sin_a - sin_b))
    if p_squared < 0:  # invalid configuration
        return None, None, None, mode
    tmp = atan2((cos_b - cos_a), d + sin_a - sin_b)
    d1 = _mod2pi(-alpha + tmp)
    d2 = sqrt(p_squared)
    d3 = _mod2pi(beta - tmp)
    return d1, d2, d3, mode


def _RSR(alpha, beta, d):
    sin_a, sin_b, cos_a, cos_b, cos_ab = _calc_trig_funcs(alpha, beta)
    mode = ["R", "S", "R"]
    p_squared = 2 + d ** 2 - (2 * cos_ab) + (2 * d * (sin_b - sin_a))
    if p_squared < 0:
        return None, None, None, mode
    tmp = atan2((cos_a - cos_b), d - sin_a + sin_b)
    d1 = _mod2pi(alpha - tmp)
    d2 = sqrt(p_squared)
    d3 = _mod2pi(-beta + tmp)
    return d1, d2, d3, mode


def _LSR(alpha, beta, d):
    sin_a, sin_b, cos_a, cos_b, cos_ab = _calc_trig_funcs(alpha, beta)
    p_squared = -2 + d ** 2 + (2 * cos_ab) + (2 * d * (sin_a + sin_b))
    mode = ["L", "S", "R"]
    if p_squared < 0:
        return None, None, None, mode
    d1 = sqrt(p_squared)
    tmp = atan2((-cos_a - cos_b), (d + sin_a + sin_b)) - atan2(-2.0, d1)
    d2 = _mod2pi(-alpha + tmp)
    d3 = _mod2pi(-_mod2pi(beta) + tmp)
    return d2, d1, d3, mode


def _RSL(alpha, beta, d):
    sin_a, sin_b, cos_a, cos_b, cos_ab = _calc_trig_funcs(alpha, beta)
    p_squared = d ** 2 - 2 + (2 * cos_ab) - (2 * d * (sin_a + sin_b))
    mode = ["R", "S", "L"]
    if p_squared < 0:
        return None, None, None, mode
    d1 = sqrt(p_squared)
    tmp = atan2((cos_a + cos_b), (d - sin_a - sin_b)) - atan2(2.0, d1)
    d2 = _mod2pi(alpha - tmp)
    d3 = _mod2pi(beta - tmp)
    return d2, d1, d3, mode


def _RLR(alpha, beta, d):
    sin_a, sin_b, cos_a, cos_b, cos_ab = _calc_trig_funcs(alpha, beta)
    mode = ["R", "L", "R"]
    tmp = (6.0 - d ** 2 + 2.0 * cos_ab + 2.0 * d * (sin_a - sin_b)) / 8.0
    if abs(tmp) > 1.0:
        return None, None, None, mode
    d2 = _mod2pi(2 * pi - acos(tmp))
    d1 = _mod2pi(alpha - atan2(cos_a - cos_b, d - sin_a + sin_b) + d2 / 2.0)
    d3 = _mod2pi(alpha - beta - d1 + d2)
    return d1, d2, d3, mode


def _LRL(alpha, beta, d):
    sin_a, sin_b, cos_a, cos_b, cos_ab = _calc_trig_funcs(alpha, beta)
    mode = ["L", "R", "L"]
    tmp = (6.0 - d ** 2 + 2.0 * cos_ab + 2.0 * d * (- sin_a + sin_b)) / 8.0
    if abs(tmp) > 1.0:
        return None, None, None, mode
    d2 = _mod2pi(2 * pi - acos(tmp))
    d1 = _mod2pi(-alpha - atan2(cos_a - cos_b, d + sin_a - sin_b) + d2 / 2.0)
    d3 = _mod2pi(_mod2pi(beta) - alpha - d1 + _mod2pi(d2))
    return d1, d2, d3, mode


_PATH_TYPE_MAP = {"LSL": _LSL, "RSR": _RSR, "LSR": _LSR, "RSL": _RSL,
                  "RLR": _RLR, "LRL": _LRL, }


def _dubins_path_planning_from_origin(end_x, end_y, end_yaw, curvature,
                                      step_size, planning_funcs):
    dx = end_x
    dy = end_y
    d = hypot(dx, dy) * curvature

    theta = _mod2pi(atan2(dy, dx))
    alpha = _mod2pi(-theta)
    beta = _mod2pi(end_yaw - theta)

    best_cost = float("inf")
    b_d1, b_d2, b_d3, b_mode = None, None, None, None

    for planner in planning_funcs:
        d1, d2, d3, mode = planner(alpha, beta, d)
        if d1 is None:
            continue

        cost = (abs(d1) + abs(d2) + abs(d3))
        if best_cost > cost:  # Select minimum length one.
            b_d1, b_d2, b_d3, b_mode, best_cost = d1, d2, d3, mode, cost

    lengths = [b_d1, b_d2, b_d3]
    x_list, y_list, yaw_list = _generate_local_course(lengths, b_mode,
                                                      curvature, step_size)

    lengths = [length / curvature for length in lengths]

    return x_list, y_list, yaw_list, b_mode, lengths


def _interpolate(length, mode, max_curvature, origin_x, origin_y,
                 origin_yaw, path_x, path_y, path_yaw):
    if mode == "S":
        path_x.append(origin_x + length / max_curvature * cos(origin_yaw))
        path_y.append(origin_y + length / max_curvature * sin(origin_yaw))
        path_yaw.append(origin_yaw)
    else:  # curve
        ldx = sin(length) / max_curvature
        ldy = 0.0
        if mode == "L":  # left turn
            ldy = (1.0 - cos(length)) / max_curvature
        elif mode == "R":  # right turn
            ldy = (1.0 - cos(length)) / -max_curvature
        gdx = cos(-origin_yaw) * ldx + sin(-origin_yaw) * ldy
        gdy = -sin(-origin_yaw) * ldx + cos(-origin_yaw) * ldy
        path_x.append(origin_x + gdx)
        path_y.append(origin_y + gdy)

        if mode == "L":  # left turn
            path_yaw.append(origin_yaw + length)
        elif mode == "R":  # right turn
            path_yaw.append(origin_yaw - length)

    return path_x, path_y, path_yaw


def _generate_local_course(lengths, modes, max_curvature, step_size):
    p_x, p_y, p_yaw = [0.0], [0.0], [0.0]

    for (mode, length) in zip(modes, lengths):
        if length == 0.0:
            continue

        # set origin state
        origin_x, origin_y, origin_yaw = p_x[-1], p_y[-1], p_yaw[-1]

        current_length = step_size
        while abs(current_length + step_size) <= abs(length):
            p_x, p_y, p_yaw = _interpolate(current_length, mode, max_curvature,
                                           origin_x, origin_y, origin_yaw,
                                           p_x, p_y, p_yaw)
            current_length += step_size

        p_x, p_y, p_yaw = _interpolate(length, mode, max_curvature, origin_x,
                                       origin_y, origin_yaw, p_x, p_y, p_yaw)

    return p_x, p_y, p_yaw


"""
Matplotlib based plotting utilities
"""
import math
import matplotlib.pyplot as plt
import numpy as np


def plot_arrow(x, y, yaw, arrow_length=1.0,
               origin_point_plot_style="xr",
               head_width=0.1, fc="r", ec="k", **kwargs):
    """
    Plot an arrow or arrows based on 2D state (x, y, yaw)

    All optional settings of matplotlib.pyplot.arrow can be used.
    - matplotlib.pyplot.arrow:
    https://matplotlib.org/stable/api/_as_gen/matplotlib.pyplot.arrow.html

    Parameters
    ----------
    x : a float or array_like
        a value or a list of arrow origin x position.
    y : a float or array_like
        a value or a list of arrow origin y position.
    yaw : a float or array_like
        a value or a list of arrow yaw angle (orientation).
    arrow_length : a float (optional)
        arrow length. default is 1.0
    origin_point_plot_style : str (optional)
        origin point plot style. If None, not plotting.
    head_width : a float (optional)
        arrow head width. default is 0.1
    fc : string (optional)
        face color
    ec : string (optional)
        edge color
    """
    if not isinstance(x, float):
        for (i_x, i_y, i_yaw) in zip(x, y, yaw):
            plot_arrow(i_x, i_y, i_yaw, head_width=head_width,
                       fc=fc, ec=ec, **kwargs)
    else:
        plt.arrow(x, y,
                  arrow_length * math.cos(yaw),
                  arrow_length * math.sin(yaw),
                  head_width=head_width,
                  fc=fc, ec=ec,
                  **kwargs)
        if origin_point_plot_style is not None:
            plt.plot(x, y, origin_point_plot_style)


def plot_curvature(x_list, y_list, heading_list, curvature,
                   k=0.01, c="-c", label="Curvature"):
    """
    Plot curvature on 2D path. This plot is a line from the original path,
    the lateral distance from the original path shows curvature magnitude.
    Left turning shows right side plot, right turning shows left side plot.
    For straight path, the curvature plot will be on the path, because
    curvature is 0 on the straight path.

    Parameters
    ----------
    x_list : array_like
        x position list of the path
    y_list : array_like
        y position list of the path
    heading_list : array_like
        heading list of the path
    curvature : array_like
        curvature list of the path
    k : float
        curvature scale factor to calculate distance from the original path
    c : string
        color of the plot
    label : string
        label of the plot
    """
    cx = [x + d * k * np.cos(yaw - np.pi / 2.0) for x, y, yaw, d in
          zip(x_list, y_list, heading_list, curvature)]
    cy = [y + d * k * np.sin(yaw - np.pi / 2.0) for x, y, yaw, d in
          zip(x_list, y_list, heading_list, curvature)]

    plt.plot(cx, cy, c, label=label)
    for ix, iy, icx, icy in zip(x_list, y_list, cx, cy):
        plt.plot([ix, icx], [iy, icy], c)


def generate_dubin_points(start, end, min_turn_radius, curvature, step_size):
    # print("Dubins path planner sample start!!")
    import matplotlib.pyplot as plt

    [start_x, start_y, start_yaw] = start
    [end_x, end_y, end_yaw] = end
    # curvature = 1/min_turn_radius

    path_x, path_y, path_yaw, mode, lengths = plan_dubins_path(start_x,
                                                               start_y,
                                                               start_yaw,
                                                               end_x,
                                                               end_y,
                                                               end_yaw,
                                                               curvature, 
                                                               step_size)

    # if show_animation:
    #     plt.scatter(path_x, path_y)#, label="".join(mode))
    #     plot_arrow(start_x, start_y, start_yaw)
    #     plot_arrow(end_x, end_y, end_yaw)
    #     plt.legend()
    #     plt.grid(True)
    #     plt.axis("equal")
    #     plt.show()

    # print(lengths)
    # print(sum(lengths))
    # print(mode)

    return [path_x, path_y, path_yaw, sum(lengths)]


if __name__ == '__main__':    
    start_x = 0.0  # [m]
    start_y = 0.0  # [m]
    # # start_yaw = np.deg2rad(67.5)  # [rad]

    # end_x = 0.0 # [m]
    # end_y = 3.0  # [m]
    # end_yaw = np.deg2rad(180)  # [rad]
    # generate_dubin_points([start_x, start_y, start_yaw], [end_x, end_y, end_yaw], 1.6, 0.1)
    
    # VALUES FOR 0 AS THETA START
    x_vals_1 = [7.0,8.0,2.0,8.0,8.0,7.0]
    y_vals_1 = [3.0,2.0,0.0,0.0,-2.0,-3.0]
    theta_vals_1 = [45,22.5,0.0,0.0,-22.5,-45]
    min_radius_vals_1 = [1.6,3.2,1.6,1.6,3.2,1.6]

    # VALUES FOR 22.5 AS THETA START
    x_vals_2 = [4.0,10.0,10.0,9.0,8.0,6.0]
    y_vals_2 = [2.0,5.0,3.0,1.0,6.0,7.0]
    theta_vals_2 = [22.5,22.5,0,-22.5,45,67.5]
    min_radius_vals_2 = [1.6,1.6,3.2,1.6,3.2,1.6]

    # # #values for 44.5 as THETA START
    x_vals_3 = [2.0,6.0,6.0,7.0,4.0,3.0]
    y_vals_3 = [2.0,6.0,4.0,3.0,6.0,7.0]
    theta_vals_3 = [45.0,45.0,22.5,0,67.5,90]
    min_radius_vals_3 = [1.6,1.6,3.2,1.6,3.2,1.6]

    #  VALUES FOR 67.5 AS THETA START
    y_vals_4 = [4.0,10.0,10.0,9.0,8.0,6.0]
    x_vals_4 = [2.0,5.0,3.0,1.0,6.0,7.0]
    theta_vals_4 = [67.5,67.5,90,112.5,45,22.5]
    min_radius_vals_4 = [1.6,1.6,3.2,1.6,3.2,1.6]

    x_vals = np.array([x_vals_1,x_vals_2,x_vals_3,x_vals_4])
    y_vals = np.array([y_vals_1,y_vals_2,y_vals_3,y_vals_4])
    theta_vals = np.array([theta_vals_1,theta_vals_2,theta_vals_3,theta_vals_4])
    min_radius_vals = np.array([min_radius_vals_1,min_radius_vals_2,min_radius_vals_3,min_radius_vals_4])

    start_angles = [0.0]#,22.5,45.0,67.5]

    backward_angles_relative = [-pi/8,0.0,pi/8]
    for k in range(0,1):#4):
        rot_angle = (pi/2)*k
        Rot_mat = np.array([[cos(rot_angle),-sin(rot_angle)],[sin(rot_angle),cos(rot_angle)]])
        for j in range(0,len(start_angles)):
            start_yaw = np.deg2rad(start_angles[j])+rot_angle
            # print([start_x,start_y,start_yaw])
            # print(start_yaw)
            for i in range(0,len(x_vals[0])):
                # print(i)
                # e_x = Rot_mat*(x_vals[j][i]*0.2)
                # e_y = Rot_mat*(y_vals[j][i]*0.2)

                [e_x,e_y] = Rot_mat@np.array([x_vals[j][i]*0.2,y_vals[j][i]*0.2])

                e_theta = np.deg2rad(theta_vals[j][i])+rot_angle
               
                rad = min_radius_vals[j][i]
                [curr_x,  curr_y, curr_yaw, curr_s] = generate_dubin_points([start_x, start_y, start_yaw], [e_x, e_y, e_theta], rad, 1/rad,0.01)
                print([start_x, start_y, pi+start_yaw], [e_x, e_y, pi+e_theta])

                plt.scatter(curr_x, curr_y)
                plot_arrow(start_x, start_y, start_yaw)
                plot_arrow(e_x, e_y, e_theta)

                for l in range(0,len(backward_angles_relative)):
                    # print(np.rad2deg(e_theta-start_yaw), backward_angles_relative[l])
                    if((e_theta-start_yaw) == backward_angles_relative[l]):
                        Rot_mat_2 = np.array([[cos(pi),-sin(pi)],[sin(pi),cos(pi)]])
                        [e_x_2,e_y_2] = Rot_mat_2@np.array([e_x,e_y])
                        [curr_x,  curr_y, curr_yaw, curr_s] = generate_dubin_points([start_x, start_y, pi+start_yaw], [e_x_2, e_y_2, pi+e_theta], rad, 1/rad,0.01)
                        # print("Start and end are : ")
                        
                        print([start_x, start_y, pi+start_yaw], [e_x_2, e_y_2, e_theta])
                        plt.scatter(curr_x, curr_y)
                        plot_arrow(start_x, start_y, start_yaw)
                        plot_arrow(e_x_2, e_y_2, e_theta)



            

        # break

    plt.legend()
    plt.grid(True)
    plt.axis("equal")
    plt.show()

    # generate_dubin_points([start_x, start_y, start_yaw], [end_x, end_y, end_yaw], 1.6, 0.1)