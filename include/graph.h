#pragma once

#include <unordered_map>

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

class Graph {
private:

public:
    unordered_map<int, Node> nodes_map;
    void add_node(int idx, Point p, double g, double h, int parent_idx, int primitive_idx);
};