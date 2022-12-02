#include "../include/map.h"

Environment::Environment(int size_x, int size_y, int num_obstacles, vector<vector<double>> map) {
    this->size_x = size_x;
    this->size_y = size_y;
    this->num_obstacles = num_obstacles;
    this->map = map;
}

void Map::create_map(std::string file_name){
    ifstream input_file;
    input_file.open(file_name, ios::in);
    std::cout<<"opened: "<<file_name<<std::endl;
    std::string line = "";
    // std::vector<std::vector<int>>map;
    this->map = map;
    while(getline(input_file, line)){
        // std::cout<<line<<std::endl;
        std::string temp_str;
        std::stringstream lineStream(line);
        std::vector<int>row;
        while(getline(lineStream, temp_str, ',')){
            // std::cout<<temp_str<<std::endl;
            row.push_back(stoi(temp_str));

        }
        map.push_back(row);
        // std::cout<<"row: "<<row.size()<<std::endl;
        row.clear();
    }
    // std::cout<<"map: "<<map.size()<<std::endl;
}


// int main(){
//     Map map;
//     map.create_map("../scripts/base_map.csv");
//     std::cout<<"Map: "<<map.map.size()<<std::endl;
//     // parseCSV();

//     return 0;
// }