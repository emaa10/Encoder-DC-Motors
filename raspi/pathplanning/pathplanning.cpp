#include "pathplanning.h"
#include <queue>
#include <ostream>
#include <iostream>


int distanceSquared(Vector& from, Vector& to) {
    Vector connectingVector = {to.x-from.x, to.y-from.y};
    if (std::abs(connectingVector.x) > 3000 || std::abs(connectingVector.y) > 2000) return 250000000;
    return connectingVector.x * connectingVector.x + connectingVector.y * connectingVector.y;
}

Pathplanner::Pathplanner(int plantsSafetyDistance, int forbiddenZonesSafetyDistance, int enemyRobotSafetyDistance, int enemyRobotRad, bool playingYellow) {
    //Set Game Variables
    this->isYellow = playingYellow;
    this->enemyDistance = robotRad + enemyRobotSafetyDistance + enemyRobotRad;
    this->plantsRad += plantsSafetyDistance + robotRad;

    /* Init forbidden Zones */

    int zoneDistance = robotRad + forbiddenZonesSafetyDistance;

    //Init playing field boundaries
    Vector topLeftCorner = {zoneDistance, 2000 - zoneDistance};
    Vector bottomRightCorner = {3000 - zoneDistance, zoneDistance};
    Vector topLeftDiagonal = {3000 - 2 * zoneDistance, -(2000 - 2 * zoneDistance)};
    Vector bottomRightDiagonal = {-topLeftDiagonal.x, - topLeftDiagonal.y};
    forbiddenZones.push_back({topLeftCorner, topLeftDiagonal});
    forbiddenZones.push_back({bottomRightCorner, bottomRightDiagonal});

    // //Init Sima Zone boundaries
    Vector simaLeftCorner = {1050 - zoneDistance, 150 + zoneDistance};
    Vector simaRightCorner = {1950 + zoneDistance, 150 + zoneDistance};
    Vector simaLeftDiagonal = {450 + zoneDistance, -(150 + zoneDistance)};
    Vector simaRightDiagonal = {-simaLeftDiagonal.x, simaLeftDiagonal.y};
    forbiddenZones.push_back({simaLeftCorner, simaLeftDiagonal});
    forbiddenZones.push_back({simaRightCorner, simaRightDiagonal});

    //Init reserved drop off area
    Vector dropOffCorner = {!isYellow * (450 + zoneDistance) + isYellow * (2550 - zoneDistance), 450 + zoneDistance};
    Vector dropOffdiagonal = {(isYellow*2-1) * (450 + zoneDistance), - (450 + zoneDistance)};
    forbiddenZones.push_back({dropOffCorner, dropOffdiagonal});

    
    /* Init plants */

    int plantCords[12][2] = {
        //Pot Stations
        {35, 612},
        {35, 1387},        
        {2965, 1387},
        {2965, 612},
        {1000, 1965},
        {2000, 1965},
        //Plant Stations        
        {1000, 700},
        {1000, 1300},
        {2000, 700},
        {2000, 1300},
        {1500, 500},
        {1500, 1500},
    };

    for (int i = 0; i < 12; i++) {
        plantGroups.push_back({plantCords[i][0], plantCords[i][1]});
    }
}

vector<Vector> Pathplanner::getPath(RobotPose from, Vector to) {
    vector<Vector> path;

    //Get enemy Position and check if the robot is inside it
    enemyPos = ldr.getEnemyPos(from);
    if (!isLegalPos(from.position) || !isLegalPos(to)) {
        return path;
    }
    
    //Calculate path
    path.push_back(from.position);
    path.push_back(to);

    
    VectorFunction function = {from.position, {to.x - from.position.x, to.y - from.position.y}};
    Vector firstObstacle = crashPoint(function);
    if (distanceSquared(from.position, firstObstacle) < distanceSquared(from.position, to)) {
        path = bfs(from.position, to);
        //Optimize the path
        // for (int i = 0; i < path.size()-1; i++) {
        //     vector<Vector> tangents = getTangentenEndpoints(path[i]);
        //     for (int j = path.size() - 1; j > i + 1; j--) {
        //         bool shortened = false;
        //         int shortestPath = calculatePathDistance(path);
        //         for (Vector t : tangents) {
        //             Vector result;
        //             if (crossLines(path[j], path[j-1], path[i], t, result)) {
        //                 vector<Vector> newpath;
        //                 for (int k = 0; k <= i; k++) {
        //                     newpath.push_back(path[k]);
        //                 }
        //                 newpath.push_back(result);
        //                 for (int k = j; k < path.size(); k++) {
        //                     newpath.push_back(path[k]);
        //                 }
        //                 if (calculatePathDistance(newpath) < shortestPath) {
        //                     path = newpath;
        //                 }
        //                 shortened = true;
        //             }
        //         }
        //     }
        // }
    }
    return path;
}

bool Pathplanner::freePath(RobotPose robot, vector<Vector> &path) {
    enemyPos = ldr.getEnemyPos(robot);

    if (path.size() < 2) return false;

    Vector v;
    VectorFunction f = {robot.position, {path[1].x - robot.position.x, path[1].y - robot.position.y}};
    bool crash = (findIntersectionWithCircle(f, enemyPos, enemyDistance, v) && distanceSquared(robot.position, v) < distanceSquared(robot.position, path[1]));

    for (int i = 1; i < path.size()-1; i++) {
        f = {path[i], {path[i+1].x - path[i].x, path[i+1].y - path[i].y}};
        crash = crash || (findIntersectionWithCircle(f, enemyPos, enemyDistance, v) && distanceSquared(path[i], v) < distanceSquared(path[i], path[i+1]));
    }
    return !crash;
}

bool Pathplanner::isLegalPos(Vector pos) {
    for (Vector &v : plantGroups) {
        if (distanceSquared(pos, v) < plantsRad*plantsRad) return false; //Inside of a plant group
    }
        
    if (distanceSquared(pos, enemyPos) <= enemyDistance*enemyDistance) return false; 
    if (forbiddenZones[0].coord.x >= pos.x || forbiddenZones[0].coord.y <= pos.y ) return false; //Outside of the playing field
    if (forbiddenZones[1].coord.x <= pos.x || forbiddenZones[1].coord.y >= pos.y ) return false; //Outside of the playing field
    if (forbiddenZones[2].coord.y >= pos.y && forbiddenZones[2].coord.x <= pos.x && forbiddenZones[3].coord.x >= pos.x) return false; //Inside the Sima Zone
    if (forbiddenZones[4].coord.y >= pos.y && (forbiddenZones[4].coord.x <= pos.x == isYellow)) return false; //Inside Reserved Drop off area
    return true;
}

bool Pathplanner::findIntersection(VectorFunction& vector1, VectorFunction& vector2, Vector& intersectionPoint) {
    double det = vector1.direction.x * vector2.direction.y - vector1.direction.y * vector2.direction.x;

    // Check if the vectors are parallel (det is zero)
    if (std::abs(det) < 1e-6) {
        return false; 
    }

    // Calculate parameters for the intersection point
    double t1 = ((vector2.start.x - vector1.start.x) * vector2.direction.y - (vector2.start.y - vector1.start.y) * vector2.direction.x) / det;
    double t2 = ((vector2.start.x - vector1.start.x) * vector1.direction.y - (vector2.start.y - vector1.start.y) * vector1.direction.x) / det;

    if (t1 < -1e-6 || t2 < -1e-6 || t2 > 1) return false; //Intersection is behind the points

    // Calculate the intersection point
    intersectionPoint.x = vector1.start.x + t1 * vector1.direction.x;
    intersectionPoint.y = vector1.start.y + t1 * vector1.direction.y;

    //Make the crash happen 1 mm before the wall
    int x = vector1.direction.x > 0 ? -1 : 1;
    int y = vector1.direction.y > 0 ? -1 : 1;

    intersectionPoint.x += x;
    intersectionPoint.y += y;

    return true; // Intersection found
}

bool Pathplanner::findIntersectionWithCircle(VectorFunction& vector1, const Vector& circleCenter, double circleRadius, Vector& intersection) {
    double a = vector1.direction.x * vector1.direction.x + vector1.direction.y * vector1.direction.y;
    double b = 2 * (vector1.direction.x * (vector1.start.x - circleCenter.x) + vector1.direction.y * (vector1.start.y - circleCenter.y));
    double c = (vector1.start.x - circleCenter.x) * (vector1.start.x - circleCenter.x) + (vector1.start.y - circleCenter.y) * (vector1.start.y - circleCenter.y) - circleRadius * circleRadius;

    // Calculate the discriminant
    double discriminant = b * b - 4 * a * c;
    if (discriminant < 0) {
        // No intersection points
        return false;
    }

    // Calculate the two possible values for t
    double t1 = (-b + std::sqrt(discriminant)) / (2 * a);
    double t2 = (-b - std::sqrt(discriminant)) / (2 * a);
    double t = abs(t2) > abs(t1) && t1 > 0 ? t1 : t2;
    if (t1 < 0 && t2 < 0) return false;

    // Calculate the intersection points
    intersection.x = vector1.start.x + t * vector1.direction.x;
    intersection.y = vector1.start.y + t * vector1.direction.y;

    return true;
}

Vector Pathplanner::crashPoint(VectorFunction &path) {
    Vector nearestIntersection;
    double dist = 25000000;

    vector<Vector> crashPoints;
    
    //Get all crashs with forbiden zones
    for (Rectangle &rect : forbiddenZones) { 
        Vector crashPoint1;
        VectorFunction f = {rect.coord, {rect.diagonal.x, 0}};
        if (findIntersection(path, f, crashPoint1)) {
            crashPoints.push_back(crashPoint1);
        }

        Vector crashPoint2;
        f = {rect.coord, {0, rect.diagonal.y}};
        if (findIntersection(path, f, crashPoint2)) {
            crashPoints.push_back(crashPoint2);
        }
    }

    //Get All crashes with plant/pot groups
    for (Vector &plant : plantGroups) {
        Vector crashPoint;
        if (findIntersectionWithCircle(path, plant, plantsRad, crashPoint)) {
            crashPoints.push_back(crashPoint);
        }
    }

    //Get crash with robot
    Vector crashPoint;
    if (findIntersectionWithCircle(path, enemyPos, enemyDistance, crashPoint)) {
        crashPoints.push_back(crashPoint);
    }


    //Select the first crash from all the crashes
    for (Vector& crash : crashPoints) {
        double crashDistance = distanceSquared(path.start, crash);
        if (crashDistance < dist) {
            dist = crashDistance;
            nearestIntersection.x = crash.x;
            nearestIntersection.y = crash.y;
        }
    }
    return nearestIntersection;
}

bool Pathplanner::crossLines(Vector &p1, Vector &p2, Vector &p3, Vector &p4, Vector &result) {
    VectorFunction f1 = {p1, {p2.x-p1.x, p2.y-p1.y}};
    VectorFunction f2 = {p2, {p1.x-p2.x, p1.y-p2.y}};
    VectorFunction f3 = {p3, {p4.x-p3.x, p4.y-p3.y}};
    VectorFunction f4 = {p4, {p3.x-p4.x, p3.y-p4.y}};
    
    if (findIntersection(f1, f3, result) && findIntersection(f2, f4, result)) {
        return true;
    }
    return false;
}


vector<Vector> Pathplanner::getTangentenEndpoints(Vector &startpoint) {
    vector<Vector> results;
    int rad = plantsRad + 5;
    for (Vector &circlePoint: plantGroups) {
        double dx = startpoint.x - circlePoint.x;
        double dy = startpoint.y - circlePoint.y;

        double distanceToCenter = std::sqrt(distanceSquared(startpoint, circlePoint));
        if (distanceToCenter <= rad) {
            continue;
        }

        // Calculate the angle between the vector and the x-axis
        double angle = std::atan2(dy, dx);
        // << angle << " " << angle * 180 / PI << endl;

        // Calculate the angle between the vector and the tangent line
        double tangentAngle = std::acos(rad/distanceToCenter);

        // Calculate the two tangent points by rotating the vector
        Vector tangent1({circlePoint.x + rad * std::cos(angle + tangentAngle),
                   circlePoint.y + rad * std::sin(angle + tangentAngle)});
        VectorFunction function1 = {startpoint, {tangent1.x-startpoint.x, tangent1.y-startpoint.y}};
        Vector tangentEnd = crashPoint(function1); 
        if (sqrt(distanceSquared(startpoint, tangentEnd)) > distanceToCenter/10) {
            results.push_back(tangentEnd);
        }

        Vector tangent2({circlePoint.x + rad * std::cos(angle - tangentAngle),
                   circlePoint.y + rad * std::sin(angle - tangentAngle)});
        VectorFunction function2 = {startpoint, {tangent2.x-startpoint.x, tangent2.y-startpoint.y}};
        tangentEnd = crashPoint(function2); 
        if (sqrt(distanceSquared(startpoint, tangentEnd)) > distanceToCenter/10) {
            results.push_back(tangentEnd);
        }
        
    }

    for (int i = 2; i < forbiddenZones.size(); i++) {
        Vector direction = {forbiddenZones[i].coord.x-startpoint.x, forbiddenZones[i].coord.y-startpoint.y + 1};
        direction.x += forbiddenZones[i].diagonal.x > 0 ? -1 : 1;
        VectorFunction edgefunction = {startpoint, direction};
        Vector edgeFunctionEnd = crashPoint(edgefunction);
        if (distanceSquared(startpoint, forbiddenZones[i].coord) < distanceSquared(startpoint, edgeFunctionEnd)) {
            results.push_back(edgeFunctionEnd);
        }
    }

    return results;
}

int Pathplanner::calculatePathDistance(vector<Vector> &path) {
    int sum = 0;
    for (int i = 0; i < path.size()-1; i++) {
        sum += distanceSquared(path[i], path[i+1]);
    }
    return sum;
}

vector<Vector> Pathplanner::bfs(Vector start, Vector end) {
    queue<vector<Vector>> queue;
    vector<Vector> firstPath;
    firstPath.push_back(start);
    queue.push(firstPath);

    //Get all Tangenten for end point
    vector<Vector> endTangenten = getTangentenEndpoints(end);

    //Data about the best path
    int nrofturns = 100;
    int length = 1410065408;
    vector<Vector> path;
    path.push_back(start);

    //Calculate route
    while (!queue.empty()) {
        vector<Vector> route = queue.front();
        queue.pop();
        if (route.size() >= nrofturns) {
            break;
        }
        

        //Calculate all Tangents
        vector<Vector> tangents = getTangentenEndpoints(route.back());
        //Check if tangents cross endpoint tangents and update best route accordingly
        bool endReached = false;
        for (Vector v1 : tangents) {
            for (Vector v2 : endTangenten) {
                Vector result;
                if (crossLines(route.back(), v1, end, v2, result)) {
                    endReached = true;
                    vector<Vector> newpath = route;
                    newpath.push_back(result);
                    int dist = calculatePathDistance(newpath);
                    if (dist < length) {
                        nrofturns = route.size();
                        length = dist;
                        path = newpath;
                    }
                }
            }
        }
        if (!endReached) {
            for (Vector v : tangents) {
                vector<Vector> newroute = route;
                newroute.push_back(v);
                queue.push(newroute);
            }
        }
    }
    path.push_back(end);
    return path;
}


void printPath(const vector<Vector>& path) {
    cout << "Path Coordinates:" << endl;
    for (const auto& point : path) {
        cout << "(" << point.x << ", " << point.y << ")" << endl;
    }
}
