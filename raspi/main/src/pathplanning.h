#pragma once

#include "structs.h"
#include <vector>

using namespace std;

#define PI 3.14159265

enum PlantGroups {
    group1 = 1,
    group2 = 0,
    group3 = 4,
    group4 = 2,
    group5 = 3,
    group6 = 5
};

enum DropOffAreas {
    reservedCorner,
    middleArea,
    upperCorner,
    reservedPlanter,
    planter1,
    planter2
};

class Pathplanner {
    private:
        // LIDAR ldr;
        bool isLegalPos(Vector pos);
        bool findIntersection(VectorFunction& vector1, VectorFunction& vector2, Vector& intersectionPoint);
        bool findIntersectionWithCircle(VectorFunction& vector1, const Vector& circleCenter, double circleRadius, Vector& intersection);
        Vector crashPoint(VectorFunction &path);
        bool crossLines(Vector &p1, Vector &p2, Vector &p3, Vector &p4, Vector &result);
        vector<Vector> getTangentenEndpoints(Vector &startpoint);
        int calculatePathDistance(vector<Vector> &path);
        vector<Vector> bfs(Vector start, Vector end);
        void shortenPath(vector<Vector>& path, int node);

    public:
        Pathplanner(int plantsSafetyDistance, int forbiddenZonesSafetyDistance, int enemyRobotSafetyDistance, int enemyRobotRad, bool playingYellow);
        vector<Vector> getPath(RobotPose from, Vector to);
        vector<Vector> getPath(RobotPose from, PlantGroups to);
        vector<Vector> getPath(RobotPose from, DropOffAreas to);
        bool freePath(RobotPose robot, vector<Vector> &path);

        int plantsRad = 125;
        vector<Vector> plantGroups;

        vector<Rectangle> forbiddenZones;

        int enemyDistance = 0;
        Vector enemyPos;

        int robotRad = 150;
        bool isYellow;
};