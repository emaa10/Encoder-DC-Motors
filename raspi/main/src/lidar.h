#pragma once

#include "sl_lidar.h" 
#include "sl_lidar_driver.h"
#include "structs.h"
#include <cmath>
#include <cstring>
#include <iostream>

using namespace sl;

#ifndef _countof
#define _countof(_Array) (int)(sizeof(_Array) / sizeof(_Array[0]))
#endif

class LIDAR {
public:
    LIDAR() {
        drv = *createLidarDriver();
        if (!drv) {
            return;
        }

        IChannel* _channel = (*createSerialPortChannel("/dev/ttyUSB0", 115200));
        sl_result result = (drv)->connect(_channel);

        sl_lidar_response_device_info_t devinfo;
        result = drv->getDeviceInfo(devinfo);
        if (!SL_IS_OK(result)) {
            return;
        }

        drv->setMotorSpeed();
        result = drv->startScan(0,1);
    }

    Vector getEnemyPos(RobotPose current_pos) {
        int collectionRough[30][20];
        std::memset(collectionRough, 0, sizeof(collectionRough));
        int collectionFine[300][200];
        std::memset(collectionFine, 0, sizeof(collectionFine));
        // std::cout << "searching enemy" << std::endl;

        sl_lidar_response_measurement_node_hq_t nodes[8192];
        size_t count = _countof(nodes);
        sl_result result = drv->grabScanDataHq(nodes, count);
        if (SL_IS_OK(result)) {
            drv->ascendScanData(nodes, count);
            for (int pos = 0; pos < (int)count ; ++pos) {
                Vector point;
                // std::cout << "found lidar scan" << std::endl;
                //Calculate point
                double angle = -(nodes[pos].angle_z_q14 * 90.f) / 16384.f - current_pos.angle;
                int distance = nodes[pos].dist_mm_q2/4.0f;
                
                double angle_rad = angle * M_PI / 180.0;
                point.x = distance * cos(angle_rad);
                point.y = distance * sin(angle_rad);
                point += current_pos.position;

                //Check if point is inside field
                if (distance < 10 || point.x < 10 || point.x > 2990 || point.y < 10 || point.y > 1990) continue;

                //Add Point to collection
                collectionRough[point.x/100][point.y/100]++;
                collectionFine[point.x/10][point.y/10]++;
            }
        }

        //Calculate Robot Position
        Vector fullestSquare;
        int maxPoints = 0;
        for (int i = 0; i < 30; i++) {
            for (int j = 0; j < 20; j++) {
                if (collectionRough[i][j] > maxPoints) {
                    fullestSquare.x = i;
                    fullestSquare.y = j;
                    maxPoints = collectionRough[i][j];
                }
            }
        }
        
        if (maxPoints == 0) {
            return {-100, -100};
        }

        return {fullestSquare.x*100, fullestSquare.y*100};

        // Vector enemyPos;
        // for (int i = fullestSquare.x*10; i < fullestSquare.x*10+10; i++) {
        //     for (int j = fullestSquare.y*10; j < fullestSquare.y*10+10; j++) {
        //         enemyPos.x += i*collectionFine[i][j]*10;
        //         enemyPos.y += j*collectionFine[i][j]*10;
        //     }
        // }
        // enemyPos.x /= maxPoints;
        // enemyPos.y /= maxPoints;

        // return enemyPos;
    }
    void stopLidar() {
        drv->stop();
        drv->disconnect();
        delete drv;
    }
private:
    ILidarDriver * drv;
};
