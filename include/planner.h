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

    double get_dubins_cost(const Point& curr_point) const;

    double get_dijkstra_cost(const Point& curr_point) const;

    double get_heuristic(const Point& curr_point, const string& method) const;

    bool goal_reached(Point& curr_point, const int& delta_x, const int& delta_y, const double& delta_theta);

    int get_index(const Point& p) const;

    Point get_xytheta(const int& idx) const;

    void expand_node(const int& idx);

    bool is_valid_cell(int a, int b) const;

public:
    Planner(Environment env);

    void search();

    void backtrack();

    void get_robot_points(vector<Point>& grid_points, vector<Point>& robot_points);
};