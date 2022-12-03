#pragma once

#include <vector>

#define GETXYTINDEX(X, Y, THETA, XSIZE, THETA_SIZE, THETA_DISC) ((int)(THETA/THETA_DISC)+THETA_SIZE*(Y*XSIZE +X))

#define GETTHETAFROMINDEX(INDEX,XSIZE,THETA_SIZE, THETA_DISC) ((INDEX%THETA_SIZE)*THETA_DISC)
#define GETXFROMINDEX(INDEX,XSIZE,THETA_SIZE, THETA_DISC) (((INDEX-(INDEX%THETA_SIZE))/THETA_SIZE)%XSIZE)
#define GETYFROMINDEX(INDEX,XSIZE,THETA_SIZE, THETA_DISC) (((INDEX-(INDEX%THETA_SIZE))/THETA_SIZE)-(((INDEX-(INDEX%THETA_SIZE))/THETA_SIZE)%XSIZE))/XSIZE

using namespace std;

struct Point {
    double x;
    double y;
    double theta;

    Point() {};
    Point(double x, double y, double theta) : x(x), y(y), theta(theta) {};

    // void operator << (const Point& p) const {
    //     cout << "x = " << p.x << ", y = " << p.y << ", theta = " << p.theta << endl;
    // }
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
