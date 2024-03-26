//#include "lidar.h"
#include "lidar.h"
#include <thread>
#include <chrono>
#include <iostream>
#include "structs.h"

int main() {
    LIDAR ldr;
    for (int i = 0; i < 100; i++) {
        //std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        Vector result = ldr.getEnemyPos({{1000,1000}, 0});
    }
    ldr.stopLidar();
}
