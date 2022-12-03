#include <iostream>
#include <vector>
#include <cfloat>
#include <cmath>

#include "./include/planner.h"
#include "./include/data.h"

using namespace std;

// FILEPATHS
string BASE_MAP_CSV = "./scripts/mit_base_map.csv";
string PRIMITIVES_JSON = "./python/mprims_dubin.json";
string WAYPOINT_TXT = "./waypoints/mit_base_map_wp.txt";

// CONFIGURATION
float DISC_THETA = 22.5;

// INPUTS
Point START(600, 10, 180);
Point GOAL(600, 10, 0);

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

    // Get the map data
    Environment m(START, GOAL, DISC_THETA);
    m.create_map(BASE_MAP_CSV);
    m.create_primitives(PRIMITIVES_JSON);
    m.check_start_goal();

    // Create a planner passing the map data
    Planner fap_planner(m);

    // Search the map with Anytime Dynamic A* or A*
    fap_planner.search();

    // Backtrack the path
    fap_planner.backtrack();

    // Print the path
    vector<Point> waypoints = fap_planner.get_robot_points();

    ofstream output_file;
    output_file.open(WAYPOINT_TXT, ios::out);

    cout << "Waypoints" << endl;
    for (unsigned int i = 0; i < waypoints.size(); i++) {
        cout << i << " : " << waypoints[i].x << ", " << waypoints[i].y << ", " << waypoints[i].theta << endl;
        output_file << i << "," << waypoints[i].x << "," << waypoints[i].y << "," << waypoints[i].theta << endl;
    }

    output_file.close();



    return 0;
}