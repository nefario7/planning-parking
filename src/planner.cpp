#include <iostream>
#include <queue>
#include <unordered_set>
#include <vector>
#include <cfloat>
#include <algorithm>
#include <math.h>

#include "../include/planner.h"
#include "../include/data.h"
#include "../include/dubins.h"

using namespace std;

// Public methods
Planner::Planner(Environment env) {
    this->env = env;

    // Get start and goal from the environment
    this->start_point = env.start_point;
    this->goal_point = env.goal_point;

    // Get start and goal idx
    this->start_idx = get_index(start_point);
    this->goal_idx = get_index(goal_point);

    // cout << "Idxs start " << start_idx << " goal " << goal_idx << endl;

    // Add start state to the graph
    graph.add_node(start_idx, start_point);
    graph.nodes_map[start_idx].g = 0.0;
    graph.nodes_map[start_idx].h = get_heuristic(start_point, HEURISTIC_METHOD);

    // Add goal state to the graph
    graph.add_node(goal_idx, goal_point);
    graph.nodes_map[goal_idx].g = FLT_MAX;
    graph.nodes_map[goal_idx].h = 0.0;
}

void Planner::search() {
    cout << "\nRunning Weighted A* search..." << endl;

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
        if (goal_reached(p, 0, 0, 0)) {
            cout << "I have reached the goal!" << endl;
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

    cout << "Total Nodes expanded = " << expansions << endl;

    if (in_closed(goal_idx))
        cout << "Found a plan" << endl;
    else
        cout << "No plan found" << endl;
    return;
}

void Planner::backtrack() {
    cout << "\nBacktracking..." << endl;
    int curr_idx = goal_idx;

    while (curr_idx != start_idx) {
        this->path.push_back(curr_idx);
        curr_idx = graph.nodes_map[curr_idx].parent_idx;
    }

    this->path.push_back(start_idx);
    reverse(this->path.begin(), this->path.end());

    return;
}

void Planner::get_robot_points(vector<Point>& grid_points, vector<Point>& robot_points) {
    cout << "Getting waypoints..." << endl;
    for (int i = 0; i < path.size(); i++) {
        int idx = path[i];
        grid_points.push_back(graph.nodes_map[idx].p);
    }

    cout << "Getting robot points..." << endl;
    for (int i = 1; i < path.size(); i++) {
        int idx = path[i];
        int p_idx = graph.nodes_map[idx].parent_idx;
        Point p = get_xytheta(p_idx);
        vector<Primitive> prims = env.primitives_map[p.theta];
        // cout << "Got primitives, idx = " << graph.nodes_map[idx].primitive_idx << endl;
        Primitive prim = prims[graph.nodes_map[idx].primitive_idx];
        for (Point p_prim : prim.primitive_points) {
            // Wrap to pi needed??
            // p_prim.x /= 0.2;
            // p_prim.y /= 0.2;
            // p_prim.theta /= 0.2;
            p_prim.x += p.x * 0.2;
            p_prim.y += p.y * 0.2;
            p_prim.theta = p.theta;
            robot_points.push_back(p_prim);
        }
    }
}

// Private methods
bool Planner::in_closed(const int& idx) const {
    if (closed.find(idx) == closed.end()) {
        return false;
    }
    return true;
}

double Planner::get_dubins_cost(const Point& curr_point) const {
    double curr[] = { curr_point.x, curr_point.y, curr_point.theta };
    double goal[] = { goal_point.x, goal_point.y, goal_point.theta };
    DubinsPath path;
    dubins_shortest_path(&path, curr, goal, 1.6);
    return path.param[0] + path.param[1] + path.param[2];;
}

double Planner::get_dijkstra_cost(const Point& curr_point) const {
    int curr_idx = GETMAPINDEX(curr_point.x, curr_point.y, env.size_x);
    return env.dijkstra_costmap.at(curr_idx);
}

double Planner::get_heuristic(const Point& curr_point, const string& method) const {
    if (method == "euclidean2D")
        return sqrt(pow(curr_point.x - goal_point.x, 2) + pow(curr_point.y - goal_point.y, 2));
    else if (method == "dubins") {
        return get_dubins_cost(curr_point);
    }
    else if (method == "dijkstra") {
        return get_dijkstra_cost(curr_point);
    }
    else if (method == "combined") {
        return max(get_dubins_cost(curr_point), get_dijkstra_cost(curr_point));
    }

    return 0;
}

bool Planner::goal_reached(Point& curr_point, const int& delta_x = 1, const int& delta_y = 1, const double& delta_theta = 22.5) {

    if (abs(curr_point.x - goal_point.x) <= delta_x && abs(curr_point.y - goal_point.y) <= delta_y
        && abs(curr_point.theta - goal_point.theta) <= delta_theta) {
        return true;
    }

    return false;
}

int Planner::get_index(const Point& p) const {
    // cout << "size theta = " << env.size_theta << endl;
    return GETXYTINDEX(p.x, p.y, p.theta, env.size_x, env.size_theta, env.disc_theta);
}

Point Planner::get_xytheta(const int& idx) const {
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
    double theta_temp = (int)(curr_point.theta * 100 + .5);
    theta_temp = (double) theta_temp / 100;

    if (env.primitives_map.find(theta_temp) == env.primitives_map.end()) {
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

            if (env.is_obstacle(check_x, check_y) || env.is_unknown(check_x, check_y)) {
                // cout << "Collision happening at " << check_x << " " << check_y << endl;
                collision = true;
                break;
            }
        }

        if (!collision && is_valid_cell(curr_point.x + primitive.end.x, curr_point.y + primitive.end.y)) {
            Point new_point(curr_point.x + primitive.end.x, curr_point.y + primitive.end.y, primitive.end.theta);
            int new_idx = get_index(new_point);

            graph.add_node(new_idx, new_point);

            if (graph.nodes_map[new_idx].g > graph.nodes_map[idx].g + primitive.cost) {
                graph.nodes_map[new_idx].g = graph.nodes_map[idx].g + primitive.cost;
                graph.nodes_map[new_idx].h = get_heuristic(new_point, HEURISTIC_METHOD);
                graph.nodes_map[new_idx].parent_idx = idx;
                if (idx < 0) {
                    cout << "Parent Index = " << idx << endl;
                    throw runtime_error("Negative Index!");
                }
                graph.nodes_map[new_idx].primitive_idx = primitive.idx;
                if (primitive.idx < 0) {
                    cout << "WARNING: Primitive is < 0 : " << primitive.idx << endl;
                }

                open.push(make_pair(graph.nodes_map[new_idx].get_f(), new_idx));
            }
        }
    }

    return;
}