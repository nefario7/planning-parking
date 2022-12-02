#include <iostream>
#include <queue>
#include <unordered_set>
#include <vector>
#include <cfloat>

#include "../include/planner.h"
#include "../include/data.h"

using namespace std;

Planner::Planner(vector<vector<int>> map_data, unordered_map<double, vector<Primitive>> primitives_map) {
    map.map = map_data;
}

void Planner::search() {

}

double Planner::get_g(int idx) {
    auto got = graph.nodes_map.find(idx);

    if (got == graph.nodes_map.end()) {
        return FLT_MAX;
    }

    else {
        return got->second.g;
    }
}

double Planner::step_cost(int idx) {
    return 1.0;
}

int Planner::get_index(int x, int y, double theta) {
    // FIX THIS
    return x * y * theta;
}

int Planner::get_heuristic(int x, int y, double theta) {
    // FIX THIS
    return 0;
}

void Planner::expand_node(int idx) {
    int x = graph.nodes_map[idx].p.x;
    int y = graph.nodes_map[idx].p.y;
    double theta = graph.nodes_map[idx].p.theta;

    auto got = primitives_map.find(theta);

    if (got == primitives_map.end()) {
        printf("Angle does not exist in primitives \n");
    }

    auto primitives = got->second;

    for (auto primitive : primitives) {
        bool primitive_flag = true;
        for (auto c : primitive.collision_cell) {
            int del_x = c.i;
            int del_y = c.j;
            int curr_x = x + del_x;
            int curr_y = y + del_y;

            if (map.is_obstacle(curr_x, curr_y)) {
                primitive_flag = false;
                break;
            }
        }
        
        Point p = {x + primitive.end.x, y + primitive.end.y, primitive.end.theta};

        int new_idx = get_index(p.x, p.y, p.theta);
        if (primitive_flag) {
            double new_g = get_g(new_idx);

            double g = get_g(idx) + step_cost(primitive.idx);
            if (new_g > g) {
                double h = get_heuristic(p.x, p.y, p.theta);
                graph.add_node(new_idx, p, g, h, idx, primitive.idx);
            }
        }
    }

    return;
}