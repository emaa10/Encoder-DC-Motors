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

    IChannel *_channel = (*createSerialPortChannel("/dev/ttyUSB1", 115200));
    sl_result result = (drv)->connect(_channel);

    sl_lidar_response_device_info_t devinfo;
    result = drv->getDeviceInfo(devinfo);
    if (!SL_IS_OK(result)) {
      std::cout << SL_IS_OK(result) << std::endl;
      return;
    }

    // checkSLAMTECLIDARHealth(drv);
    drv->setMotorSpeed();
    result = drv->startScan(0, 1);
  }

  bool checkSLAMTECLIDARHealth(ILidarDriver *drv) {
    sl_result op_result;
    sl_lidar_response_device_health_t healthinfo;

    op_result = drv->getHealth(healthinfo);
    if (SL_IS_OK(op_result)) { // the macro IS_OK is the preperred way to judge
                               // whether the operation is succeed.
      std::cout << "SLAMTEC Lidar health status : " << healthinfo.status
                << std::endl;
      if (healthinfo.status == SL_LIDAR_STATUS_ERROR) {
        std::cout << "some error on lidar" << std::endl;
        // enable the following code if you want slamtec lidar to be reboot by
        // software
        drv->reset();
        return false;
      } else {
        return true;
      }

    } else {
      std::cout << "Error, cannot retrieve the lidar health code: " << op_result
                << std::endl;
      return false;
    }
  }

  Vector getEnemyPos(RobotPose current_pos) {
    int collectionRough[30][20];
    std::memset(collectionRough, 0, sizeof(collectionRough));
    int collectionFine[300][200];
    std::memset(collectionFine, 0, sizeof(collectionFine));

    sl_lidar_response_measurement_node_hq_t nodes[8192];
    size_t count = _countof(nodes);
    sl_result result = drv->grabScanDataHq(nodes, count);
    if (SL_IS_OK(result)) {
      drv->ascendScanData(nodes, count);
      for (int pos = 0; pos < (int)count; ++pos) {
        Vector point;
        // Calculate point
        double angle =
            -(nodes[pos].angle_z_q14 * 90.f) / 16384.f - current_pos.angle;
        int distance = nodes[pos].dist_mm_q2 / 4.0f;

        double angle_rad = angle * M_PI / 180.0;
        point.x = distance * cos(angle_rad);
        point.y = distance * sin(angle_rad);
        point += current_pos.position;

        // Check if point is inside field
        if (distance < 10 || point.x < 10 || point.x > 2990 || point.y < 10 ||
            point.y > 1990)
          continue;

        // Add Point to collection
        collectionRough[point.x / 100][point.y / 100]++;
        collectionFine[point.x / 10][point.y / 10]++;
      }
    }

    // Calculate Robot Position
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
      return {0, 0};
    }

    return {fullestSquare.x * 100, fullestSquare.y * 100};
  }

  bool freeFront(RobotPose current_pos) {
    sl_lidar_response_measurement_node_hq_t nodes[8192];
    size_t count = _countof(nodes);
    sl_result result = drv->grabScanDataHq(nodes, count);
    if (SL_IS_OK(result)) {
      drv->ascendScanData(nodes, count);
      int enemyCounter = 0;
      for (int pos = 0; pos < (int)count; ++pos) {
        Vector point;
        // Calculate point
        int distance = nodes[pos].dist_mm_q2 / 4.0f;
        double rel_angle = -(nodes[pos].angle_z_q14 * 90.f) / 16384.f + 180;
        rel_angle = fmod(rel_angle + 360.0, 360.0);
        double angle = rel_angle + current_pos.angle;
        angle = fmod(angle + 360.0, 360.0);
        // std::cout << "ANGLE: " << angle << std::endl;

        double angle_rad = angle * M_PI / 180.0;
        point.x = distance * cos(angle_rad);
        point.y = distance * sin(angle_rad);
        point += current_pos.position;

        // Check if point is inside field
        if (distance < 100 || point.x < 100 || point.x > 2900 || point.y < 100 ||
            point.y > 1900)
          continue;

        if (distance < 400 && (rel_angle > 300 || rel_angle < 60)) {
          //std::cout << "enemy: x: " << point.x << ", y: " << point.y << "angle" << rel_angle << std::endl;
          //return false;
          enemyCounter++;
        }
      }
      return enemyCounter < 10;
    }
    return true;
  }

  bool freeTurn(RobotPose current_pos) {
    sl_lidar_response_measurement_node_hq_t nodes[8192];
    size_t count = _countof(nodes);
    sl_result result = drv->grabScanDataHq(nodes, count);
    if (SL_IS_OK(result)) {
      drv->ascendScanData(nodes, count);
      int enemyCounter = 0;
      for (int pos = 0; pos < (int)count; ++pos) {
        Vector point;
        // Calculate point
        double angle =
            (nodes[pos].angle_z_q14 * 90.f) / 16384.f + current_pos.angle;
        angle = fmod(angle + 360.0, 360.0);
        int distance = nodes[pos].dist_mm_q2 / 4.0f;

        double angle_rad = angle * M_PI / 180.0;
        point.x = distance * cos(angle_rad);
        point.y = distance * sin(angle_rad);
        point += current_pos.position;

        // Check if point is inside field
        if (distance < 10) //|| point.x < 10 || point.x > 2990 || point.y < 10 ||
        //     point.y > 1990)
          continue;

        // Check if enemy is near
        if (distance < 400)
          enemyCounter++;
      }
      return enemyCounter < 10;
    }
    return true;
  }

  bool freeBack(RobotPose current_pos) {
    sl_lidar_response_measurement_node_hq_t nodes[8192];
    size_t count = _countof(nodes);
    sl_result result = drv->grabScanDataHq(nodes, count);
    if (SL_IS_OK(result)) {
      drv->ascendScanData(nodes, count);
      int enemyCounter = 0;
      for (int pos = 0; pos < (int)count; ++pos) {
        Vector point;
        // Calculate point
        double rel_angle = -(nodes[pos].angle_z_q14 * 90.f) / 16384.f + 180;
        rel_angle = fmod(rel_angle + 360.0, 360.0);
        double angle = rel_angle + current_pos.angle;
        angle = fmod(angle + 360.0, 360.0);
        // std::cout << "ANGLE: " << angle << std::endl;
        int distance = nodes[pos].dist_mm_q2 / 4.0f;

        double angle_rad = angle * M_PI / 180.0;
        point.x = distance * cos(angle_rad);
        point.y = distance * sin(angle_rad);
        point += current_pos.position;

        // Check if point is inside field
        if (distance < 10 || point.x < 150 || point.x > 2850 || point.y < 150 ||
            point.y > 1850)
          continue;

        // Check if enemy is near
        if (distance < 400 && rel_angle > 120 && rel_angle < 240) {
          // std::cout << "enemy back: x: " << point.x << ", y: " << point.y << "angle" << rel_angle << std::endl;
          // return false;
          enemyCounter++;
        }
      }
      return enemyCounter < 10;
    }
    return true;
  }

  void stopLidar() {
    drv->stop();
    drv->disconnect();
    delete drv;
  }

private:
  ILidarDriver *drv;
};