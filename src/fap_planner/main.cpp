#include <iostream>
#include <vector>
#include <cfloat>
#include <cmath>

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <nav_msgs/Odometry.h>

#include "./include/planner.h"
#include "./include/data.h"

using namespace std;

// FILEPATHS
// string MAP_NAME = "b_level";
string MAP_NAME = "mit_base";
string BASE_MAP_CSV = "./src/fap_planner/maps/" + MAP_NAME + "_map.csv";
string PARKING_MAP_CSV = "./src/fap_planner/maps/" + MAP_NAME + "_traversability_map.csv";
string PRIMITIVES_JSON = "./src/fap_planner/python/mprims_dubin.json";
string WAYPOINT_TXT = "./src/fap_planner/waypoints/" + MAP_NAME + "_map_wp.txt";
string PARKING_WAYPOINT_TXT = "./src/fap_planner/waypoints/" + MAP_NAME + "_map_parkwp.txt";
string ROBOTPOINT_TXT = "./src/fap_planner/waypoints/" + MAP_NAME + "_map_rp.txt";

// CONFIGURATION
float DISC_THETA = 22.5;
int MIN_X = -290;
int MIN_Y = -17;

// INPUTS
// Point START(190, 10, 157.5);
// Point GOAL(190, 115, 45);

// Point START(600, 10, 180);
Point START(380, 143, 0);
Point GOAL(125, 150, 90);

// ROS global variables
bool planner_called = false;
bool park_called = false;
Point start;
Point goal;
Point current;

void plannerStartHandler(const geometry_msgs::PoseArray::ConstPtr &msg)
{
    planner_called = true;
    start.x = (int)msg->poses[0].position.x;
    start.y = (int)msg->poses[0].position.y;
    start.theta = msg->poses[0].position.z;
    goal.x = (int)msg->poses[1].position.x;
    goal.y = (int)msg->poses[1].position.y;
    goal.theta = msg->poses[1].position.z;
}

void parkStartHandler(const geometry_msgs::Pose::ConstPtr &msg)
{
    park_called = true;
    current.x = (int)msg->position.x;
    current.y = (int)msg->position.y;
    current.theta = msg->position.z;
}

int main(int argc, char **argv)
{
    // ROS
    if (argc == 1)
    {
        cout << "No arguments given. Using ROS by default." << endl;
        ros::init(argc, argv, "fap_planner");
        ros::NodeHandle nh;
        ros::NodeHandle nhPrivate = ros::NodeHandle("~");

        // Subscribers and publishers (advertisers)
        ros::Subscriber robot_startgoal_positions = nh.subscribe<geometry_msgs::PoseArray>("/vector", 5, plannerStartHandler);
        ros::Subscriber robot_current_position = nh.subscribe<geometry_msgs::Pose>("/park_start", 5, parkStartHandler);
        ros::Publisher full_path_pub = nh.advertise<geometry_msgs::PoseArray>("/fap_planner/full_path", 5);
        ros::Publisher safe_path_pub = nh.advertise<geometry_msgs::PoseArray>("/fap_planner/safe_path", 5);
        ros::Rate r(1);

        cout << "----------- Environment -----------" << endl;
        Environment m(start, goal, DISC_THETA);
        m.create_map(BASE_MAP_CSV);
        m.create_parking_costmap(PARKING_MAP_CSV);
        m.create_primitives(PRIMITIVES_JSON);

        while (ros::ok())
        {
            ros::spinOnce();

            if (planner_called)
            {
                planner_called = false;

                cout << "----------- Checking Start and Goal Points -----------" << endl;
                m.start_point = start;
                m.goal_point = goal;
                if (!m.check_start_goal())
                {
                    r.sleep();
                    continue;
                }

                cout << "----------- Backward Dijkstra -----------" << endl;
                m.compute_dijkstra_costmap();

                cout << "----------- Planner -----------" << endl;
                // Create a planner passing the map data
                Planner fap_planner(m, 1.0, "combined", false);

                // Search the map with Anytime Dynamic A* or A*
                if (fap_planner.search())
                {
                    // Backtrack the path
                    fap_planner.backtrack();

                    cout << "----------- Waypoints -----------" << endl;
                    vector<Point> waypoints;
                    vector<Point> robot_points;
                    fap_planner.get_robot_points(waypoints, robot_points, MIN_X + 1, MIN_Y + 1);

                    ofstream output_file;
                    output_file.open(WAYPOINT_TXT, ios::out);
                    for (unsigned int i = 0; i < waypoints.size(); i++)
                    {
                        cout << i << " : " << waypoints[i].x << ", " << waypoints[i].y << ", " << waypoints[i].theta << endl;
                        output_file << i << "," << waypoints[i].x << "," << waypoints[i].y << "," << waypoints[i].theta << endl;
                    }
                    output_file.close();

                    geometry_msgs::PoseArray robot_pose_array;
                    for (unsigned int i = 0; i < robot_points.size(); i++)
                    {
                        geometry_msgs::Pose robot_pose;
                        robot_pose.position.x = robot_points[i].x;
                        robot_pose.position.y = robot_points[i].y;
                        robot_pose.position.z = robot_points[i].theta;
                        robot_pose_array.poses.push_back(robot_pose);
                    }
                    full_path_pub.publish(robot_pose_array);
                }
            }

            else if (park_called)
            {
                park_called = false;

                // Run the parking search to find the best planner
                Planner parking_planner(m, current, vector<float>{0.5, 1.0, 1.5, 2.0}, 20000, true);

                if (parking_planner.search())
                {
                    // Backtrack the path
                    parking_planner.backtrack();

                    cout << "----------- Waypoints -----------" << endl;
                    vector<Point> parking_spot_waypoints;
                    vector<Point> robot_points;
                    parking_planner.get_robot_points(parking_spot_waypoints, robot_points, MIN_X + 1, MIN_Y + 1);

                    ofstream output_file;
                    output_file.open(PARKING_WAYPOINT_TXT, ios::out);
                    for (unsigned int i = 0; i < parking_spot_waypoints.size(); i++)
                    {
                        cout << i << " : " << parking_spot_waypoints[i].x << ", " << parking_spot_waypoints[i].y << ", " << parking_spot_waypoints[i].theta << endl;
                        output_file << i << "," << parking_spot_waypoints[i].x << "," << parking_spot_waypoints[i].y << "," << parking_spot_waypoints[i].theta << endl;
                    }
                    output_file.close();

                    geometry_msgs::PoseArray robot_pose_array;
                    for (unsigned int i = 0; i < robot_points.size(); i++)
                    {
                        geometry_msgs::Pose robot_pose;
                        robot_pose.position.x = robot_points[i].x;
                        robot_pose.position.y = robot_points[i].y;
                        robot_pose.position.z = robot_points[i].theta;
                        robot_pose_array.poses.push_back(robot_pose);
                    }
                    safe_path_pub.publish(robot_pose_array);
                }
            }
            r.sleep();
        }
    }
    else if (argc > 1 && argv[1] == "noros")
    {
        // Get the map data
        cout << "----------- Environment -----------" << endl;
        Environment m(START, GOAL, DISC_THETA);
        m.create_map(BASE_MAP_CSV);
        m.create_parking_costmap(PARKING_MAP_CSV);
        m.create_primitives(PRIMITIVES_JSON);

        cout << "----------- Checking Start and Goal Points -----------" << endl;
        if (!m.check_start_goal())
            throw runtime_error("Start or goal point is not valid.");
        m.compute_dijkstra_costmap();

        cout << "----------- Planner -----------" << endl;
        // Create a planner passing the map data
        Planner scanstar_planner(m, 1.0, "combined", false);
        // Search the map with Anytime Dynamic A* or A*
        if (scanstar_planner.search())
        {
            // Backtrack the path
            scanstar_planner.backtrack();

            cout << "----------- Waypoints -----------" << endl;
            vector<Point> waypoints;
            vector<Point> robot_points;
            scanstar_planner.get_robot_points(waypoints, robot_points, MIN_X + 1, MIN_Y + 1);

            ofstream output_file;
            output_file.open(WAYPOINT_TXT, ios::out);
            for (unsigned int i = 0; i < waypoints.size(); i++)
            {
                cout << i << " : " << waypoints[i].x << ", " << waypoints[i].y << ", " << waypoints[i].theta << endl;
                output_file << i << "," << waypoints[i].x << "," << waypoints[i].y << "," << waypoints[i].theta << endl;
            }
            output_file.close();
        }

        // Run the parking search to find the best planner
        Planner parking_planner(m, current, vector<float>{0.5, 1.0, 1.5, 2.0}, 50000, true);

        if (parking_planner.search())
        {
            // Backtrack the path
            parking_planner.backtrack();

            cout << "----------- Waypoints -----------" << endl;
            vector<Point> parking_spot_waypoints;
            vector<Point> robot_points;
            parking_planner.get_robot_points(parking_spot_waypoints, robot_points, MIN_X + 1, MIN_Y + 1);

            ofstream output_file;
            output_file.open(PARKING_WAYPOINT_TXT, ios::out);
            for (unsigned int i = 0; i < parking_spot_waypoints.size(); i++)
            {
                cout << i << " : " << parking_spot_waypoints[i].x << ", " << parking_spot_waypoints[i].y << ", " << parking_spot_waypoints[i].theta << endl;
                output_file << i << "," << parking_spot_waypoints[i].x << "," << parking_spot_waypoints[i].y << "," << parking_spot_waypoints[i].theta << endl;
            }
            output_file.close();
        }
    }

    return 0;
}