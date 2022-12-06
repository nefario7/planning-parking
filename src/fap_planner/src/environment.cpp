#include <queue>
#include <unordered_set>
#include <vector>
#include <cmath>
#include <cfloat>

#include "../include/environment.h"
#include "../include/data.h"
#include "../include/rapidjson/document.h"
#include "../include/rapidjson/writer.h"
#include "../include/rapidjson/stringbuffer.h"

#include "../include/rapidjson/istreamwrapper.h"

int dX[NUMOFDIRS] = { -1, -1, -1, 0, 0, 1, 1, 1 };
int dY[NUMOFDIRS] = { -1, 0, 1, -1, 1, -1, 0, 1 };
double cost[NUMOFDIRS] = { 0.2*sqrt(2), 0.2, 0.2*sqrt(2), 0.2, 0.2, 0.2*sqrt(2), 0.2, 0.2*sqrt(2) };

using namespace std;
using namespace rapidjson;

struct HNode {
    int x, y;
    double g = FLT_MAX;

    HNode() {}
    HNode(int x, int y, double g) : x(x), y(y), g(g) {}
};

Environment::Environment() {}

Environment::Environment(Point start, Point goal, float disc_theta) {
    start_point = start;
    goal_point = goal;
    this->disc_theta = disc_theta;
}

void Environment::create_map(string file_name) {
    ifstream input_file;
    input_file.open(file_name, ios::in);

    if (!input_file.is_open()) {
        cout << "Error opening file" << endl;
        return;
    }

    cout << "Creating map from : " << file_name << endl;
    string line = "";

    while (getline(input_file, line)) {
        // cout<<line<<endl;
        string temp_str;
        stringstream lineStream(line);
        vector<int>row;
        while (getline(lineStream, temp_str, ',')) {
            // cout<<temp_str<<endl;
            row.push_back(stoi(temp_str));

        }
        map.push_back(row);
        // cout<<"row: "<<row.size()<<endl;
        row.clear();
    }
    // cout<<"map: "<<map.size()<<endl;

    size_x = map.size();
    size_y = map[0].size();
    size_theta = (int)(360 / disc_theta);

    cout << "Map Sizes = " << size_x << " " << size_y << " " << size_theta << endl;
}

bool Environment::is_obstacle(const int& a, const int& b) const {
    if (map[a][b] == 1)
        return true;
    return false;
}

bool Environment::is_openspace(const int& a, const int& b) const {
    if (map[a][b] == 0)
        return true;
    return false;
}

bool Environment::is_unknown(const int& a, const int& b) const {
    if (map[a][b] == -1)
        return true;
    return false;
}

double Environment::get_cost(const int& idx) const {
    double backward_cost_mult = 1.0;
    switch(idx) {
        case 0:
        case 6:
            return 2.0;
        case 1:
        case 5:
            return 1.5;
        case 2:
            return 1.0;
        case 3:
            return 1.75;
        case 4:
            return 3.0;
        case 7:
        case 9:
            return 2.0 * backward_cost_mult;
        case 8:
            return 1.0 * backward_cost_mult;
        default:
            return 3.0;
    }
    throw runtime_error("Cost doesn't map to an index");
    return 1.0;
}

void Environment::compute_dijkstra_costmap() {
    unordered_map <int, HNode> cell_map;

    priority_queue<pair<double, int>, vector<pair<double, int>>, greater<pair<double, int>>> open;

    int goal_idx = GETMAPINDEX(goal_point.x, goal_point.y, size_x);
    cell_map.insert(make_pair(goal_idx, HNode(goal_point.x, goal_point.y, 0)));
    open.push(make_pair(0, goal_idx));

    while (!open.empty()) {
        int current_idx = open.top().second;
        open.pop();

        for (int i = 0; i < NUMOFDIRS; i++) {
            int new_x = GETXFROM2DINDEX(current_idx, size_x) + dX[i];
            int new_y = GETYFROM2DINDEX(current_idx, size_x) + dY[i];

            if (new_x < 0 || new_x >= size_x || new_y < 0 || new_y >= size_y)
                continue;

            if (is_obstacle(new_x, new_y) || is_unknown(new_x, new_y))
                continue;

            int new_idx = GETMAPINDEX(new_x, new_y, size_x);

            double new_g = cell_map[current_idx].g + cost[i];

            if (cell_map.find(new_idx) == cell_map.end()) {
                cell_map.insert(make_pair(new_idx, HNode(new_x, new_y, new_g)));
                open.push(make_pair(new_g, new_idx));
            }
            else if (new_g < cell_map[new_idx].g) {
                cell_map[new_idx].g = new_g;
                open.push(make_pair(new_g, new_idx));
            }

            dijkstra_costmap.insert(make_pair(new_idx, new_g));
        }
    }

    // Save the costmap to a csv file
    // ofstream output_file;
    // output_file.open("costmap.csv", ios::out);

    // if (!output_file.is_open()) {
    //     cout << "Error opening file" << endl;
    //     return;
    // }

    // for (int i = 0; i < size_x; i++) {
    //     for (int j = 0; j < size_y; j++) {
    //         int idx = GETMAPINDEX(i, j, size_x);
    //         if (dijkstra_costmap.find(idx) != dijkstra_costmap.end()) {
    //             output_file << dijkstra_costmap[idx] << ',';
    //         }
    //         else {
    //             output_file << -1 << ',';
    //         }
    //     }
    //     output_file << endl;
    // }

    // output_file.close();
}

void Environment::create_primitives(const string file_name) {
    ifstream input_file;
    input_file.open(file_name, ios::in);

    if (!input_file.is_open()) {
        cout << "Error opening file" << endl;
        return;
    }

    cout << "Loading primitives from : " << file_name << endl;

    // Read json with rapidjson and store primitives in a map
    IStreamWrapper isw(input_file);

    Document d;
    d.ParseStream(isw);

    // const Value& p = primitives["0"];
    // const Value& start = p["start"];


    // Read nested json using rapidjson

    for (Value::ConstMemberIterator itr = d.MemberBegin(); itr != d.MemberEnd(); ++itr) {
        string key = itr->name.GetString();
        double angle = stod(key);

        const Value& primitives = d[key.c_str()];

        vector<Primitive> primitive_list;
        for (int i = 0; i < NUMOFPRIMS; i++) {
            string s = to_string(i);
            const Value& p = primitives[s.c_str()];
            const Value& start = p["start"];
            const Value& end = p["end"];

            const Value& primitive_points = p["mprim"];
            const Value& collision_cells = p["collisions"];

            vector<Point> prim_points;
            const Value& x_vals = primitive_points[0];
            const Value& y_vals = primitive_points[1];
            const Value& theta_vals = primitive_points[2];
            for (SizeType k = 0; k < x_vals.Size(); k++) {
                Point temp_point(x_vals[k].GetDouble(), y_vals[k].GetDouble(), theta_vals[k].GetDouble());
                prim_points.push_back(temp_point);
            }

            vector<Cell> prim_cells;
            const Value& i_vals = collision_cells[0];
            const Value& j_vals = collision_cells[1];
            for (SizeType k = 0; k < i_vals.Size(); k++) {
                Cell temp_cell(i_vals[k].GetInt(), j_vals[k].GetInt());
                prim_cells.push_back(temp_cell);
            }

            double cost = get_cost(i);

            Primitive prim = Primitive{
                Point{start[0].GetDouble(), start[1].GetDouble(), start[2].GetDouble()},
                Point{end[0].GetDouble(), end[1].GetDouble(), end[2].GetDouble()},
                prim_points,
                prim_cells,
                i,
                cost
            };

            primitive_list.push_back(prim);
        }

        primitives_map.insert(make_pair(angle, primitive_list));
    }

    compute_dijkstra_costmap();
}

bool Environment::check_start_goal() {
    if (!is_openspace(start_point.x, start_point.y))
    {
        printf("Invalid start point, point not in open space! \n");
        return false;
    }

    if (!is_openspace(goal_point.x, goal_point.y))
    {
        printf("Invalid goal point, point not in open space! \n");
        return false;
    }

    return true;
}

// int main() {
//     Environment env;
//     env.create_primitives("../python/mprims_new.json");

//     for (auto& p : env.primitives_map[11.25]) {
//         cout << "Start: " << p.start.x << ", " << p.start.y << ", " << p.start.theta << endl;
//         cout << "End: " << p.end.x << ", " << p.end.y << ", " << p.end.theta << endl;
//         cout << "Primitive points: " << p.primitive_points.size() << endl;
//         cout << "Collision cells: " << p.collision_cells.size() << endl;
//     }

//     return 0;
// }