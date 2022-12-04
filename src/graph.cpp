#include <iostream>

#include "../include/graph.h"

using namespace std;

Node::Node(Point p, double g, double h, int parent_idx, int primitive_idx) {
    this->p = p;
    this->g = g;
    this->h = h;
    this->parent_idx = parent_idx;
    this->primitive_idx = primitive_idx;
}

double Node::get_f() const {
    return this->g + WEIGHT * this->h;
}

// Add a node to the graph
void Graph::add_node(int idx, Point p, double g, double h, int parent_idx, int primitive_idx) {
    nodes_map.insert(make_pair(idx, Node(p, g, h, parent_idx, primitive_idx)));
    return;
}

// Add a node to the graph [Default g = FLT_MAX, h = 0.0]
void Graph::add_node(int idx, Point p) {
    nodes_map.insert(make_pair(idx, Node(p, FLT_MAX, 0.0, -1, -1)));
    return;
}