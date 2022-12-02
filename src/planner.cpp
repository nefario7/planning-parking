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

bool Planner::isClosed(const int s_idx) {
    if (graph.CLOSED.find(s_idx) == graph.CLOSED.end()) {
        return false;
    }
    return true;
}

int Planner::getFScoreIdx() {
    OPENLIST_F OP_F = graph.OPEN_F.top();
    graph.OPEN_F.pop();
    return OP_F.i;
}

void Planner::insertClosed(const int s_idx) {
    graph.CLOSED.insert(s_idx);
}

void Planner::search() {
    // FIX THIS
    int start_x, start_y;
    double start_theta;
    int goal_idx;

    Point p = {start_x, start_y, start_theta};
    int init_idx = get_index(p.x, p.y, p.theta);
    double h = get_heuristic(p.x, p.y, p.theta);
    double g = get_g(init_idx);
    graph.add_node(init_idx, p, g, h, -1, -1);

    while(!isClosed(goal_idx) && !graph.OPEN_F.empty())
    {
        int curr_idx = getFScoreIdx();

        int curr_x, curr_y;
        double curr_theta;
        getXYZFromIdx(curr_idx, curr_x, curr_y, curr_theta);

        if (isClosed(curr_idx))
        {
            continue;
        }

        insertClosed(curr_idx);

        expand_node(curr_idx);
    }

    printf("\n");
    if (isClosed(goal_idx))
    {
        printf("Found a plan \n");
    }
    else
    {
        printf("No plan found \n");
    }
    return;
}


double Planner::get_g(int idx) {
    auto got = graph.OPEN.find(idx);

    if (got == graph.OPEN.end()) {
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

void Planner::getXYZFromIdx(int idx, int &x, int &y, double &theta) {
    // FIX THIS
    return;
}

int Planner::get_heuristic(int x, int y, double theta) {
    // FIX THIS
    return 0;
}

void Planner::expand_node(int idx) {
    int x = graph.OPEN[idx].p.x;
    int y = graph.OPEN[idx].p.y;
    double theta = graph.OPEN[idx].p.theta;

    auto got = primitives_map.find(theta);

    if (got == primitives_map.end()) {
        printf("Angle does not exist in primitives \n");
    }

    auto primitives = got->second;

    for (auto primitive : primitives) {
        if (primitive.start.theta == theta) {
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
    }

    return;
}