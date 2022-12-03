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
    // int size_theta;

    Point start_point;
    Point goal_point;

    // Primitives data
    unordered_map<double, std::vector<Primitive>> primitives_map;

    void create_map(std::string filename);
    bool is_obstacle(int a, int b);
    bool is_open(int a, int b);
    bool is_unknown(int a, int b);

    void create_primitives(std::string filename);

};


// ---------------------------------------