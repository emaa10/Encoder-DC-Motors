//#include "lidar.h"
#include "lidar.h"
#include <thread>
#include <chrono>
#include <iostream>
#include "structs.h"

int main() {
    LIDAR ldr;
    while(1) { std::cout << ldr.freeFront({{500, 500}, 0}) << std::endl; }
   ldr.stopLidar();
}
