#include <iostream>
#include <vector>
#include <cfloat>
#include <cmath>

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>

#include "./include/planner.h"
#include "./include/data.h"

using namespace std;

// FILEPATHS
// string MAP_NAME = "b_level";
string MAP_NAME = "mit_base";
string BASE_MAP_CSV = "./src/fap_planner/maps/" + MAP_NAME +"_map.csv";
string PRIMITIVES_JSON = "./src/fap_planner/python/mprims_dubin.json";
string WAYPOINT_TXT = "./src/fap_planner/waypoints/" + MAP_NAME +"_map_wp.txt";
string ROBOTPOINT_TXT = "/home/nambags/mmpug_ugv/" + MAP_NAME +"_map_rp.txt";

// CONFIGURATION
float DISC_THETA = 22.5;
int MIN_X = -290;
int MIN_Y = -17;

// INPUTS
// Point START(190, 10, 157.5);
// Point GOAL(190, 115, 45);

// Point START(600, 10, 180);
Point START(100, 10, 0);
Point GOAL(200, 187, 180);


// ROS global variables
bool planner_called = false;



void plannerStartHandler(const std_msgs::Bool::ConstPtr& msg)
{
    if (msg->data)
    {
        planner_called = true;
    }
}


int main(int argc, char** argv) {
    // ROS
    ros::init(argc, argv, "fap_planner");
    ros::NodeHandle nh;
    ros::NodeHandle nhPrivate = ros::NodeHandle("~");

    // Subscribers and publishers (advertisers)
    ros::Subscriber planner_flag = nh.subscribe<std_msgs::Bool> ("/fap_planner", 5, plannerStartHandler);

    ros::Publisher full_path_pub = nh.advertise<geometry_msgs::PoseArray> ("/fap_planner/full_path", 5);


    ros::Rate r(1);

    while(ros::ok()) {
        ros::spinOnce();

        if (planner_called) {
            planner_called = false;

            cout << "----------- Environment -----------" << endl;
            // Get the map data
            Environment m(START, GOAL, DISC_THETA);
            m.create_map(BASE_MAP_CSV);
            m.create_primitives(PRIMITIVES_JSON);
            m.check_start_goal();

            cout << "----------- Planner -----------" << endl;
            // Create a planner passing the map data
            Planner fap_planner(m);

            // Search the map with Anytime Dynamic A* or A*
            fap_planner.search();

            // Backtrack the path
            fap_planner.backtrack();

            cout << "----------- Waypoints -----------" << endl;
            // Print the path
            vector<Point> waypoints;
            vector<Point> robot_points;
            fap_planner.get_robot_points(waypoints, robot_points, MIN_X + 1, MIN_Y + 1);

            ofstream output_file;
            output_file.open(WAYPOINT_TXT, ios::out);
            for (unsigned int i = 0; i < waypoints.size(); i++) {
                cout << i << " : " << waypoints[i].x << ", " << waypoints[i].y << ", " << waypoints[i].theta << endl;
                output_file << i << "," << waypoints[i].x << "," << waypoints[i].y << "," << waypoints[i].theta << endl;
            }


            geometry_msgs::PoseArray robot_pose_array;

            for (unsigned int i = 0; i < robot_points.size(); i++) {
                geometry_msgs::Pose robot_pose;
                robot_pose.position.x = robot_points[i].x;
                robot_pose.position.y = robot_points[i].y;
                robot_pose.position.z = robot_points[i].theta;
                robot_pose_array.poses.push_back(robot_pose);
            }
            full_path_pub.publish(robot_pose_array);

            // ofstream output_file_rp;
            // output_file_rp.open(ROBOTPOINT_TXT, ios::out);

            // for (unsigned int i = 0; i < robot_points.size(); i++) {
            //     output_file_rp << i << "," << robot_points[i].x << "," << robot_points[i].y << "," << robot_points[i].theta << endl;
            // }

            output_file.close();
            // output_file_rp.close();
        }

        r.sleep();
    }


    return 0;
}