#pragma once

#include <unordered_map>
#include <queue>
#include <unordered_set>
#include <cfloat>

#include "data.h"

using namespace std;

struct Node {
    Point p;
    double g;
    double h;
    int parent_idx;
    int primitive_idx;

    Node() {};
    Node(Point p, double g, double h, int parent_idx, int primitive_idx);

    double get_f() const;
};

struct Graph {
    unordered_map<int, Node> nodes_map;

    void add_node(int idx, Point p, double g, double h, int parent_idx, int primitive_idx);

    void add_node(int idx, Point p);
};