#include <iostream>
#include <vector>

#include "./include/planner.h"
#include "./include/data.h"

using namespace std;

string BASE_MAP_CSV = "./scripts/mit_base_map.csv";
string PRIMITIVES_JSON = "./python/mprims_new.json";

// CONFIGURATION
float DISC_THETA = 22.5;

int main() {
    // Read the configuration file
    //! Need to do this

    // Get the map data
    Environment m;
    m.create_map(BASE_MAP_CSV);
    m.create_primitives(PRIMITIVES_JSON);
    m.disc_theta = DISC_THETA;

    // Create a planner passing the map data
    Planner fap_planner(m);

    // Search the map with Anytime Dynamic A* or A*
    fap_planner.search();

    // Backtrack the path

    // Print the path

    return 0;
}