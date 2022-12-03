#pragma once

#include <vector>

#define GETXYTINDEX(X, Y, THETA, XSIZE,THETA_SIZE, THETA_DISC) ((int)(THETA/THETA_DISC)+THETA_SIZE*(Y*XSIZE +X))

using namespace std;

struct Point {
    double x;
    double y;
    double theta;

    Point() {};
    Point(double x, double y, double theta) : x(x), y(y), theta(theta) {};
};

struct Cell {
    int i;
    int j;

    Cell() {};
    Cell(int i, int j) : i(i), j(j) {};
};

struct Primitive {
    Point start;
    Point end;
    vector<Point> primitive_points;
    vector<Cell> collision_cells;
    int idx;
};
