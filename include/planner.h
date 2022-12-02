#pragma once

#include <iostream>
#include <string>
#include <fstream>
#include <vector>
#include <queue>
#include <unordered_map>
#include <unordered_set>

#include "graph.h"
#include "map.h"

using namespace std;

class Planner {
private:
    Graph graph;
    Map map;
    // vector<vector<int>> map;
    unordered_map<double, vector<Primitive>> primitives_map;    // robot_angle : vector<Primitive>

    void expand_node(int idx);

    double get_g(int idx);
    
    double step_cost(int idx);

    int get_index(int x, int y, double theta);

    int get_heuristic(int x, int y, double theta);

public:
    Planner(vector<vector<int>> map_data, unordered_map<double, vector<Primitive>> primitives_map);

    void search();

    void backtrack();
};