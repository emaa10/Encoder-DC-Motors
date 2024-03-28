// #include "lidar.h"
#include "pathplanning.h"
#include <thread>
#include <chrono>
#include <iostream>
#include "structs.h"
#include <unistd.h>
static inline void delay(sl_word_size_t ms) {
    while (ms >= 1000) {
        usleep(1000 * 1000);
        ms -= 1000;
    };
    if (ms != 0)
        usleep(ms * 1000);
}


int main() {
    Pathplanner p(-20, 0,0,200,true);
    std::vector<Vector> path = p.getPath({{0, 0}, 0}, {300,300});

    while(true) {
        std::cout << p.freePath({{0, 1000}, 0}, path) << std::endl;
    }
}
