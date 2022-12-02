#pragma once

#include <unordered_map>

using namespace std;

struct Node {
    int x, y;
    double theta;
    double g;
    double h;
    int parent_idx;
    int primitive_idx;

    Node() {};
    Node(int x, int y, double theta, double g, double h);

    double get_f() const;
};

class Graph {
    private:
    unordered_map<int, Node> nodes_map;

    public:
    void add_node(int idx, int x, int y, double theta, double g, double h);

    void expand_node();
};