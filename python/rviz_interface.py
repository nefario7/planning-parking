#!/usr/bin/env python
from glob import glob
from re import X
import rospy
import ros_numpy
from sensor_msgs.msg import PointCloud2 as pc2
from std_msgs.msg import String, Float64MultiArray, Float32
import matplotlib.pyplot as plt
import numpy as np
from numpy import savetxt
from nav_msgs.msg import Path
from geometry_msgs.msg import Pose

class rviz_interface():
    def __init__(self):
        self.waypoint_sub = rospy.Subscriber("/basestation/rviz/waypoints", Path, self.way_point_CB)
        self.vector_pub = rospy.Publisher("/vector", Pose, queue_size=1)
        self.pose = Pose()

        rospy.loginfo("Waiting for input")

    def way_point_CB(self, msg):

        try:
            x1 = (int)(msg.poses[0].pose.position.x * 5) # for discretization
            y1 = (int)(msg.poses[0].pose.position.y * 5) # for discretization

            x2 = (int)(msg.poses[1].pose.position.x * 5) # for discretization
            y2 = (int)(msg.poses[1].pose.position.y * 5) # for discretization

            vector = [x2-x1, y2-y1]
            theta = np.floor(np.arctan2(vector[1], vector[0]) % (2*np.pi) / (np.pi/8)) # for discretization
            theta = np.rad2deg(theta * (np.pi/8)) # converting to degrees

            self.pose.position.x = x1
            self.pose.position.y = y1
            self.pose.position.z = theta
            self.vector_pub.publish(self.pose)
            rospy.loginfo("Received input: %f, %f, %f", x1, y1, theta)
            rospy.loginfo("Waiting for new input")

        except:
            rospy.logerr("Please select exactly 2 waypoints")  
     

    def run(self):
        rospy.spin()

if __name__=='__main__':
    rospy.init_node('rviz_interface')
    node = rviz_interface()
    node.run()