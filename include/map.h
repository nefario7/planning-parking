#pragma once

#include <vector>

using namespace std;

struct Environment {
    int size_x;
    int size_y;
    int num_obstacles;
    vector<vector<double>> map;

    Environment(int size_x, int size_y, int num_obstacles, vector<vector<double>> map);
};