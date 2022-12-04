#include <vector>
#include <string>
#include <fstream>
#include <sstream>
#include <iostream>

using namespace std;

void read_waypoint(string file_name, vector<vector<double>> & waypoints){
    ifstream input_file;
    input_file.open(file_name, ios::in);

    if (!input_file.is_open()) {
        cout << "Error opening file" << endl;
        return;
    }

    cout << "Reading waypoints from : " << file_name << endl;
    string line = "";
    // vector<vector<int>>waypoints;

    while (getline(input_file, line)) {
        // cout<<line<<endl;
        string temp_str;
        stringstream lineStream(line);
        vector<double>row;
        while (getline(lineStream, temp_str, ',')) {
            // cout<<temp_str<<endl;
            row.push_back(std::stod(temp_str));

        }
        waypoints.push_back(row);
        // cout<<"row: "<<row.size()<<endl;
        row.clear();
    }

}

int main(){
    vector<vector<double>>waypoints;
    read_waypoint("../scripts/mit_base_map_wp_turn2.txt", waypoints);
    // cout<<"size:\n"<<waypoints.size()<<std::endl;
    for (auto & it: waypoints){
        for (auto & itt: it){
            cout<<itt<<",";
        }
        cout<<""<<endl;
    }
    return 0;
}