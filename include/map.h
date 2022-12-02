#pragma once

#include <vector>
#include <string>
#include <fstream>
#include <sstream>
#include <iostream>

using namespace std;

struct Environment {
    int size_x;
    int size_y;
    int num_obstacles;
    vector<vector<double>> map;

    Environment(int size_x, int size_y, int num_obstacles, vector<vector<double>> map);
};

// ---------------------------------------

struct Map{
    std::vector<std::vector<int>>map;
    void create_map(std::string filename);
    bool is_obstacle(int a, int b); 
    bool is_open(int a, int b);
    bool is_unknown(int a, int b);
    double get_cost(int a, int b);
};


// ---------------------------------------