#include "../include/map.h"

Environment::Environment(int size_x, int size_y, int num_obstacles, vector<vector<double>> map) {
    this->size_x = size_x;
    this->size_y = size_y;
    this->num_obstacles = num_obstacles;
    this->map = map;
}