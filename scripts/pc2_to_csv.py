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

def subscriberCallBack(msg):
    global count
    global global_xyz
    global iter
    print(iter)
    iter = iter + 1
    count = count + 1
    xyz_array = ros_numpy.point_cloud2.pointcloud2_to_xyz_array(msg)

    print("array size: ", xyz_array.shape)
    file_name = "../maps/xyz_raw_3d.csv"
    savetxt(file_name, xyz_array, delimiter=",")
    print("Saved raw array!!")


    # global_xyz = np.vstack((global_xyz, xyz_array[:,0:3]))
    # print("global_xyz_first: ", global_xyz.shape)
    row_idx = np.asarray(np.where(np.logical_and(xyz_array[:,2]>0.3, xyz_array[:,2]<2.2)))
    # print("row_idx: ", row_idx.shape)
    # # print("xyz: ",xyz_array[row_idx,0:3].shape)
    # print("global second: ", global_xyz.shape)
    xyz_filtered = xyz_array[row_idx[0,:], 0:4]

    print("xyz_filtered: ", xyz_filtered.shape)
    file_name = "xyz_filtered_3d.csv"
    savetxt(file_name, xyz_array, delimiter=",")
    print("Saved filtered array!!")


    # global_xyz = np.vstack((global_xyz, xyz_array[:,0:3]))
    # print("global_xyz_first: ", global_xyz.shape)
    # row_idx = np.asarray(np.where(np.logical_and(global_xyz[:,2]>0.3, global_xyz[:,2]<2.2)))
    # print("row_idx: ", row_idx.shape)
    # # print("xyz: ",xyz_array[row_idx,0:3].shape)
    # print("global second: ", global_xyz.shape)
    # global_xyz = global_xyz[row_idx[0,:], 0:4]
    # print("golbal: ", global_xyz.shape)




    # print('count: ',count)
    # if count == 5:
    #     count = 0
    #     ax = plt.axes()
    #     ax.set(facecolor = "black")
    #     row_idx = np.where(np.logical_and(global_xyz[:,2]>0.5, global_xyz[:,2]<1)) #play aroud with limits to include/exclude the floor/roof
    #     # plt.scatter(global_xyz[row_idx,0], global_xyz[row_idx,1], s=0.001, c='red', marker = '.')
    #     # plt.xlim([-30,20])
    #     # plt.ylim([-30,20])
    #     # name = '/home/p43s/test_ws/src/kmz_test/scripts/' + 'recent_map' + '.png'   # path and name where image is stored. should be same as that in talker.py
    #     name = '/home/p43s/test_ws/src/kmz_test/scripts/' + 'nb-slam_run_1-1-' +str(iter) + '.png' 
    #     # plt.savefig(name, dpi = 600, bbox_inches='tight', pad_inches=0, facecolor='black')
    #     global_xyz = np.zeros((0,3))
    # np.save("/home/p43s/test_ws/src/kmz_test/scripts/nb-slam_run_1-partial-cropped.npy", global_xyz)
    # print("saved!!")
    
def listener():
    topic_name = "/basestation/map_compress/full_cloud/3d"
    rospy.init_node('subscriberNode', anonymous=True)
    rospy.Subscriber(topic_name, pc2, subscriberCallBack) #change topic name as required
    rospy.spin()


if __name__ == '__main__':

    listener()