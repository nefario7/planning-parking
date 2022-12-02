#pragma once
#include <vector>

using namespace std;

struct Point {
    double x;
    double y;
    double theta;
};

struct Cell {
    int i;
    int j;
};

struct Primitive {
    Point start;
    Point end;
    vector<Point> primitive_points;
    vector<Cell> collision_cell;
    int idx;
};
