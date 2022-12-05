#!/usr/bin/env python
from glob import glob
from re import X
import rospy
import ros_numpy
from sensor_msgs.msg import PointCloud2 as pc2
from std_msgs.msg import String
import matplotlib.pyplot as plt
import numpy as np
from numpy import savetxt

global_xyz = np.zeros((0,3))
count = 0
iter = 0

# FILE_NAME = "b_level"
FILE_NAME = "mit_base"

def subscriberCallBack(msg):
    global count
    global global_xyz
    global iter
    print(iter)
    iter = iter + 1
    count = count + 1
    xyz_array = ros_numpy.point_cloud2.pointcloud2_to_xyz_array(msg)

    print("array size: ", xyz_array.shape)
    file_name = f"../maps/{FILE_NAME}_3d.csv"
    savetxt(file_name, xyz_array, delimiter=",")
    print("Saved raw array!!")

    # row_idx = np.asarray(np.where(np.logical_and(xyz_array[:,2]>0.3, xyz_array[:,2]<2.2)))
    # xyz_filtered = xyz_array[row_idx[0,:], 0:4]

    # print("xyz_filtered: ", xyz_filtered.shape)
    # file_name = "xyz_filtered_3d.csv"
    # savetxt(file_name, xyz_array, delimiter=",")
    # print("Saved filtered array!!")
    
def listener():
    topic_name = "/basestation/map_compress/full_cloud/3d"
    rospy.init_node('subscriberNode', anonymous=True)
    rospy.Subscriber(topic_name, pc2, subscriberCallBack) #change topic name as required
    rospy.spin()


if __name__ == '__main__':

    listener()