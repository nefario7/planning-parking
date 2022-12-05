#include <iostream>
#include <vector>
#include <cfloat>
#include <cmath>

#include "./include/planner.h"
#include "./include/data.h"

using namespace std;

// FILEPATHS
// string MAP_NAME = "b_level";
string MAP_NAME = "mit_base";
string BASE_MAP_CSV = "./maps/" + MAP_NAME +"_map.csv";
string PRIMITIVES_JSON = "./python/mprims_dubin.json";
string WAYPOINT_TXT = "./waypoints/" + MAP_NAME +"_map_wp.txt";
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

// this->start_point = Point(43, 10, 0);
// this->start_point = Point(600, 10, 180);
// this->start_point = Point(200, 205, 180);
// this->goal_point = Point(358, 99, 45);
// this->goal_point = Point(43, 10, 180);
// this->goal_point = Point(600, 10, 90);
// this->goal_point = Point(200, 205, 90);


int main() {
    // Read the configuration file
    //! Need to do this

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

    ofstream output_file_rp;
    output_file_rp.open(ROBOTPOINT_TXT, ios::out);

    for (unsigned int i = 0; i < robot_points.size(); i++) {
        output_file_rp << i << "," << robot_points[i].x << "," << robot_points[i].y << "," << robot_points[i].theta << endl;
    }

    output_file.close();
    output_file_rp.close();

    for (unsigned int i = 0; i < robot_points.size(); i++) {
        output_file_rp << i << "," << robot_points[i].x << "," << robot_points[i].y << "," << robot_points[i].theta << endl;
    }

    output_file.close();
    output_file_rp.close();

    return 0;
}