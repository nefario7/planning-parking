#pragma once

#include <unordered_map>
#include <queue>
#include <unordered_set>

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

struct OPENLIST_F{
    int i;
    float f;

    OPENLIST_F(int i_, float f_) 
        : i(i_), f(f_)
    {
    }
};

struct FMIN{
    bool operator()(OPENLIST_F const& op1, OPENLIST_F const& op2) {
        return op1.f > op2.f;
    }
};

class Graph {
private:

public:
    unordered_set<int> CLOSED;
    priority_queue<OPENLIST_F, vector<OPENLIST_F>, FMIN> OPEN_F;
    unordered_map<int, Node> OPEN;
    void add_node(int idx, Point p, double g, double h, int parent_idx, int primitive_idx);
};