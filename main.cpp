#include <iostream>
#include <vector>

#include "./include/planner.h"
#include "./include/data.h"

using namespace std;

string BASE_MAP = "../scripts/base_map.csv";

int main() {
    // Get the map data
    Map m;
    m.create_map(BASE_MAP);

    unordered_map<double, vector<Primitive>> primitives_map;

    // Create a planner passing the map data
    Planner fap_planner(m.map, primitives_map);

    // Search the map with Anytime Dynamic A* or A*
    fap_planner.search();

    // Backtrack the path

    // Print the path

    return 0;
}