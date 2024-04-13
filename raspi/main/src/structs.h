#pragma once

#include <atomic>

struct Vector {
    int x, y;

    Vector& operator+=(const Vector& rhs)
    {
        x += rhs.x;
        y += rhs.y;
        return *this; 
    }    
};

struct RobotPose {
    Vector position;
    double angle;
};

struct Rectangle {
    Vector coord, diagonal;
};

struct VectorFunction {
    Vector start, direction;
};


