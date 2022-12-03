#pragma once

#include <iostream>
#include <string>
#include <fstream>
#include <vector>
#include <queue>
#include <unordered_map>
#include <unordered_set>

#include "graph.h"
#include "environment.h"

using namespace std;

class Planner {
private:
    Graph graph;
    Environment env;
    // vector<vector<int>> map;
    // unordered_map<double, vector<Primitive>> primitives_map;    // robot_angle : vector<Primitive>

    // Start and goal points
    int start_idx, goal_idx;
    Point start_point, goal_point;

    // A* lists
    unordered_set<int> closed;
    priority_queue<pair<double, int>, vector<pair<double, int>>, greater<pair<double, int>>> open;

    // Backtracked path
    vector<int> path;
    vector<Point> robot_points;

    bool in_closed(const int& idx) const;

    double get_heuristic(Point& curr_point, const string& method) const;

    double step_cost(int idx);

    int get_index(const Point& p) const;

    // void getXYZFromIdx(int idx, int& x, int& y, double& theta);

    void expand_node(const int& idx);

    bool is_valid_cell(int a, int b) const;

public:
    Planner(Environment env);

    void search();

    void backtrack();

    vector<Point> get_robot_points();
};