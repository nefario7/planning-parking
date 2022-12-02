#include <iostream>
#include <vector>

#include "./include/planner.h"
#include "./include/data.h"

using namespace std;

string BASE_MAP_CSV = "../scripts/base_map.csv";
string PRIMITIVES_JSON = "../python/mprims_new.json";

int main() {
    // Get the map data
    Environment m;
    m.create_map(BASE_MAP_CSV);
    m.create_primitives(PRIMITIVES_JSON);

    // Create a planner passing the map data
    Planner fap_planner(m);

    // Search the map with Anytime Dynamic A* or A*
    fap_planner.search();

    // Backtrack the path

    // Print the path

    return 0;
}