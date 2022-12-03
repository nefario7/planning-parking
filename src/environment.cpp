#include <cmath>

#include "../include/environment.h"
#include "../include/data.h"
#include "../include/rapidjson/document.h"
#include "../include/rapidjson/writer.h"
#include "../include/rapidjson/stringbuffer.h"

#include "../include/rapidjson/istreamwrapper.h"

using namespace std;
using namespace rapidjson;

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
    size_theta = (int) (360 / disc_theta);
    cout << "disc theta = " << disc_theta << endl;
    cout << "map sizes = " << size_x << " " << size_y << " " << size_theta << endl;
}

bool Environment::is_obstacle(int a, int b) {
    if (map[a][b] == 1)
        return true;
    else
        return false;
}

bool Environment::is_open(int a, int b) {
    if (map[a][b] == 0) {
        return true;
    }
    else {
        return false;
    }
}

bool Environment::is_unknown(int a, int b) {
    if (map[a][b] == -1) {
        return true;
    }
    else {
        return false;
    }
}

void Environment::create_primitives(const string file_name) {
    ifstream input_file;
    input_file.open(file_name, ios::in);

    if (!input_file.is_open()) {
        cout << "Error opening file" << endl;
        return;
    }

    cout << "Create primitives called \n";

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
        for (int i = 0; i < 7; i++) {
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

            primitive_list.push_back(Primitive{
                Point{start[0].GetDouble(), start[1].GetDouble(), start[2].GetDouble()},
                Point{end[0].GetDouble(), end[1].GetDouble(), end[2].GetDouble()},
                prim_points,
                prim_cells
                });
        }

        primitives_map.insert(make_pair(angle, primitive_list));
    }
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