#include <iostream>
#include <queue>
#include <unordered_set>
#include <vector>
#include <cfloat>
#include <algorithm>
#include <math.h>

#include "../include/planner.h"
#include "../include/data.h"

using namespace std;

// Public methods
Planner::Planner(Environment env) {
    this->env = env;

    //! Get start and goal from the environment
    this->start_point = Point(43, 10, 0);
    this->goal_point = Point(358, 99, 90);

    if (!env.is_open(43, 10) && !env.is_open(360, 100)) {
        throw runtime_error("BC sahi points do!");
    }

    // Get start and goal idx
    this->start_idx = get_index(start_point);
    this->goal_idx = get_index(goal_point);

    cout << "Idxs start " << start_idx << " goal " << goal_idx << endl;


    // Add start state to the graph
    graph.add_node(start_idx, start_point);
    graph.nodes_map[start_idx].g = 0.0;
    graph.nodes_map[start_idx].h = get_heuristic(start_point, "euclidean2D");

    // Add goal state to the graph
    graph.add_node(goal_idx, goal_point);
    graph.nodes_map[goal_idx].g = FLT_MAX;
    graph.nodes_map[goal_idx].h = 0.0;
}

void Planner::search() {
    cout << "Running Weighted A* search..." << endl;

    // Add start state to the open list
    open.push(make_pair(graph.nodes_map[start_idx].get_f(), start_idx));

    int expansions = 0;
    while (!open.empty() && !in_closed(goal_idx)) {
        // Get the index of the node with the lowest f score
        double curr_f = open.top().first;
        int curr_idx = open.top().second;
        open.pop();

        // Get the node with the lowest f score
        Node curr_node = graph.nodes_map[curr_idx];

        Point p = get_xytheta(curr_idx);
        if (get_heuristic(p, "euclidean2D") < 2){
            cout << "Fuck man, I have reached the goal!" << endl;
            cout << p.theta << endl;
            goal_idx = curr_idx;
        }

        // Add the node to the closed list if not already there
        if (in_closed(curr_idx))
            continue;
        closed.insert(curr_idx);

        // Expand the node
        expand_node(curr_idx);
        expansions++;
        if (expansions % 10000 == 0) {
            cout << "Expanded " << expansions << " nodes" << endl;
        }
    }

    if (in_closed(goal_idx))
        cout << "Found a plan \n";
    else
    {
        cout << "Nodes expanded = " << expansions;
        printf("No plan found \n");
    }
    return;
}

void Planner::backtrack() {
    cout << "Backtracking..." << endl;
    int curr_idx = goal_idx;

    while (curr_idx != start_idx) {
        this->path.push_back(curr_idx);
        curr_idx = graph.nodes_map[curr_idx].parent_idx;
    }

    this->path.push_back(start_idx);
    reverse(this->path.begin(), this->path.end());

    return;
}

vector<Point> Planner::get_robot_points() {
    cout << "Getting robot points..." << endl;
    for (int i = 0; i < path.size(); i++) {
        int idx = path[i];
        robot_points.push_back(graph.nodes_map[idx].p);
    }

    return robot_points;
}

// Private methods
bool Planner::in_closed(const int& idx) const {
    if (closed.find(idx) == closed.end()) {
        return false;
    }
    return true;
}

double Planner::get_heuristic(Point& curr_point, const string& method) const {
    if (method == "euclidean2D")
        return sqrt(pow(curr_point.x - goal_point.x, 2) + pow(curr_point.y - goal_point.y, 2));
    else if (method == "euclidean3D")
        return sqrt(pow(curr_point.x - goal_point.x, 2) + pow(curr_point.y - goal_point.y, 2) + pow(curr_point.theta - goal_point.theta, 2));
    return 0;
}

double Planner::step_cost(int idx) {
    return 1.0;
}

int Planner::get_index(const Point &p) const {
    // cout << "size theta = " << env.size_theta << endl;
    return GETXYTINDEX(p.x, p.y, p.theta, env.size_x, env.size_theta, env.disc_theta);
}

Point Planner::get_xytheta(const int &idx) const {
    Point p;
    p.x = GETXFROMINDEX(idx, env.size_x, env.size_theta, env.disc_theta);
    p.y = GETYFROMINDEX(idx, env.size_x, env.size_theta, env.disc_theta);
    p.theta = GETTHETAFROMINDEX(idx, env.size_x, env.size_theta, env.disc_theta);

    return p;
}

bool Planner::is_valid_cell(const int a, const int b) const {
    if (a < 0 || b < 0 || a >= env.size_x || b >= env.size_y) {
        return false;
    }
    return true;
}

void Planner::expand_node(const int& idx) {
    Point curr_point = graph.nodes_map[idx].p;

    // cout << "expanding " << idx << endl;

    // Get the primitives for the current angle
    if (env.primitives_map.find(curr_point.theta) == env.primitives_map.end()) {
        printf("Angle does not exist in primitives \n");
        printf("Current theta is %f \n", curr_point.theta);
        throw runtime_error("Angle does not exist in primitives");
    }

    const vector<Primitive> curr_primitives(env.primitives_map[curr_point.theta]);
    for (const auto& primitive : curr_primitives) {
        bool collision = false;
        for (auto c : primitive.collision_cells) {
            int check_x = (int)(curr_point.x + c.i);
            int check_y = (int)(curr_point.y + c.j);

            if (!is_valid_cell(check_x, check_y)) continue;

            if (env.is_obstacle(check_x, check_y)) {
                // cout << "Collision happening at " << check_x << " " << check_y << endl;
                collision = true;
                break;
            }
        }

        if (!collision) {
            Point new_point(curr_point.x + primitive.end.x, curr_point.y + primitive.end.y, primitive.end.theta);
            int new_idx = get_index(new_point);

            graph.add_node(new_idx, new_point);

            if (graph.nodes_map[new_idx].g > graph.nodes_map[idx].g + step_cost(primitive.idx)) {
                graph.nodes_map[new_idx].g = graph.nodes_map[idx].g + step_cost(primitive.idx);
                graph.nodes_map[new_idx].h = get_heuristic(new_point, "euclidean2D");
                graph.nodes_map[new_idx].parent_idx = idx;
                if (idx < 0){
                    cout << "Parent Index = " << idx << endl;
                    throw runtime_error("Negative Index!");
                }
                graph.nodes_map[new_idx].primitive_idx = primitive.idx;

                open.push(make_pair(graph.nodes_map[new_idx].get_f(), new_idx));
            }
        }
    }

    return;
}