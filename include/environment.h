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
    // Map data
    std::vector<std::vector<int>>map;
    int size_x;
    int size_y;
    int size_theta;
    float disc_theta;

    Point start_point;
    Point goal_point;

    // Primitives data
    unordered_map<double, std::vector<Primitive>> primitives_map;

    Environment();

    Environment(Point start, Point goal, float disc_theta);

    void create_map(std::string filename);

    bool is_obstacle(const int& a, const int& b) const;

    bool is_openspace(const int& a, const int& b) const;

    bool is_unknown(const int& a, const int& b) const;

    void create_primitives(std::string filename);

    bool check_start_goal();

};


// ---------------------------------------