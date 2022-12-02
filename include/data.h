#pragma once
#include <vector>

using namespace std;

struct Point {
    double x;
    double y;
    double theta;

    Point(double x, double y, double theta) : x(x), y(y), theta(theta) {}
};

struct Cell {
    int i;
    int j;

    Cell(int i, int j) : i(i), j(j) {}
};

struct Primitive {
    Point start;
    Point end;
    vector<Point> primitive_points;
    vector<Cell> collision_cell;
    int idx;
};
