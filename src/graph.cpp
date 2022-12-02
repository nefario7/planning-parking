#include <iostream>

#include "../include/graph.h"

using namespace std;

Node::Node(Point p, double g, double h, int parent_idx = -1, int primitive_idx = -1) {
    this->p = p;
    this->g = g;
    this->h = h;
    this->parent_idx = parent_idx;
    this->primitive_idx = primitive_idx;
}

double Node::get_f() const {
    return this->g + this->h;
}

void Graph::add_node(int idx, Point p, double g, double h, int parent_idx = -1, int primitive_idx = -1) {
    OPEN.insert(make_pair(idx, Node(p, g, h, parent_idx, primitive_idx)));
    return;
}