#pragma once

#include <vector>
#include <string>
#include <fstream>
#include <sstream>
#include <iostream>
#include <unordered_map>

#include "data.h"

using namespace std;

class Environment {
public:
    // Obstacle Map data
    vector<std::vector<int>>map;

    // Parking map data
    vector<vector<double>>parking_costmap;

    // Map data
    int size_x;
    int size_y;
    int size_theta;
    float disc_theta;

    Point start_point;
    Point goal_point;

    // Primitives data
    unordered_map<double, std::vector<Primitive>> primitives_map;

    // Dijkstra Costs
    unordered_map<int, double> dijkstra_costmap;

    Environment();

    Environment(Point start, Point goal, float disc_theta);

    void create_map(const string filename);

    void create_parking_costmap(const string filename);

    void create_primitives(const string filename);

    bool is_obstacle(const int& a, const int& b) const;

    bool is_openspace(const int& a, const int& b) const;

    bool is_unknown(const int& a, const int& b) const;

    double get_cost(const int& idx) const;

    void compute_dijkstra_costmap();

    bool check_start_goal();

};


// ---------------------------------------