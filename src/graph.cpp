#include <iostream>

#include "../include/graph.h"

using namespace std;

Node::Node(Point p, double g, double h) {
    this->p = p;
    this->g = g;
    this->h = h;
    this->parent_idx = -1;
    this->primitive_idx = -1;
}

double Node::get_f() const {
    return this->g + this->h;
}

void Graph::add_node(int idx, Point p, double g, double h) {
    nodes_map.insert(make_pair(idx, Node(p, g, h)));
    return;
}





