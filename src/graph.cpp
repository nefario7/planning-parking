#include <iostream>

#include "../include/graph.h"

using namespace std;

Node::Node(int x, int y, double theta, double g, double h) {
    this->x = x;
    this->y = y;
    this->theta = 0;
    this->g = g;
    this->h = h;
    this->parent_idx = -1;
    this->primitive_idx = -1;
}

double Node::get_f() const {
    return this->g + this->h;
}

void Graph::add_node(int idx, int x, int y, double theta, double g, double h) {
    nodes_map.insert(make_pair(idx, Node(x, y, theta, g, h)));
    return;
}

void Graph::expand_node() {

    return;
}





