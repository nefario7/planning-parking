#!/usr/bin/env python
import rospy
import numpy as np
from std_msgs.msg import Bool
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import Pose, PoseArray
from tf.transformations import euler_from_quaternion

# Map offset
MIN_X = -290
MIN_Y = -17
# Initial start position - MIN
OFFSET_X = - MIN_X * 0.2
OFFSET_Y = - MIN_Y * 0.2

class rviz_interface():
    def __init__(self):
        self.odometry_sub = rospy.Subscriber("/rc1/integrated_to_init", Odometry, self.odometryCallback)
        self.waypoint_sub = rospy.Subscriber("/basestation/rviz/waypoints", Path, self.wayPointCB)
        self.planner_flag = rospy.Subscriber("/fap_planner", Bool, self.plannerFlagCB)

        self.vector_pub = rospy.Publisher("/vector", PoseArray, queue_size=1)
        self.start_goal = PoseArray()
        self.odom = Odometry()
        self.flag = False
        self.got_waypoint = False

        rospy.loginfo("Waiting for input")

    def odometryCallback(self, msg):
        self.odom = msg
    
    def wayPointCB(self, msg):
        if self.flag:
            self.start_goal.poses.clear()
            try:
                x1 = (int)((msg.poses[0].pose.position.x + OFFSET_X) * 5)
                y1 = (int)((msg.poses[0].pose.position.y + OFFSET_Y) * 5)

                x2 = (int)((msg.poses[1].pose.position.x + OFFSET_X) * 5)
                y2 = (int)((msg.poses[1].pose.position.y + OFFSET_Y) * 5)

                vector = [x2-x1, y2-y1]
                theta = np.floor(np.arctan2(vector[1], vector[0]) % (2*np.pi) / (np.pi/8))
                theta = self.wrap_to_360(np.rad2deg(theta * (np.pi/8)))

                x0 = (int)((self.odom.pose.pose.position.x + OFFSET_X) * 5)
                y0 = (int)((self.odom.pose.pose.position.y + OFFSET_Y) * 5)
                orientation_q = self.odom.pose.pose.orientation
                orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
                (_, _, yaw) = euler_from_quaternion (orientation_list)
                yaw = np.floor(yaw / (np.pi/8))
                yaw = self.wrap_to_360(np.rad2deg(yaw * (np.pi/8)))

                start_pose = Pose()
                start_pose.position.x = x0
                start_pose.position.y = y0
                start_pose.position.z = yaw
                self.start_goal.poses.append(start_pose)
                goal_pose = Pose()
                goal_pose.position.x = x1
                goal_pose.position.y = y1
                goal_pose.position.z = theta
                self.start_goal.poses.append(goal_pose)

                rospy.loginfo("Received start input: %f, %f, %f", x0, y0, yaw)
                rospy.loginfo("Received goal input: %f, %f, %f", x1, y1, theta)
                rospy.loginfo("Waiting for new input")

                self.got_waypoint = True

            except:
                rospy.logerr("Please select exactly 2 waypoints")  
    
    def plannerFlagCB(self, msg):
        self.flag = True
        self.got_waypoint = False
    
    def wrap_to_360(self, angle):
        while angle >= 360:
            angle -= 360
        while angle < 0:
            angle += 360
        return angle

    def run(self):
        r = rospy.Rate(1)
        while not rospy.is_shutdown():

            if (self.flag and self.got_waypoint):
                self.got_waypoint = False
                self.flag = False
                self.vector_pub.publish(self.start_goal)
            
            r.sleep()


if __name__=='__main__':
    rospy.init_node('rviz_interface')
    node = rviz_interface()
    node.run()