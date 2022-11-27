#pragma once

#include <iostream>
#include <string>
#include <fstream>
#include <vector>
#include <queue>
#include <unordered_map>
#include <unordered_set>

#include "map.h"

using namespace std;

class Planner {
private:

public:
    Planner();

    void search();

    void backtrack();
};