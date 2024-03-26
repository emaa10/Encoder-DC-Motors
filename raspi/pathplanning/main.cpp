//#include "lidar.h"
#include "pathplanning.h"
#include <thread>
#include <chrono>
#include <iostream>
#include "structs.h"

int main() {
    Pathplanner p(-20, 0,0,200,true);
    std::vector<Vector> path = p.getPath({{0, 1000}, 0}, {1500,1000});
    std::cout << p.freePath({{0, 1000}, 0}, path);
}