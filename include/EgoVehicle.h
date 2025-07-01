#ifndef _EGO_VEHICLE_H
#define _EGO_VEHICLE_H

#include <config.h>

#include <cmath>
#include <opencv2/core.hpp>
#include <string>
#include <tuple>
using namespace Config;

float calculateTargetSpeed(float distance, float frontSpeed, float egoSpeed,
                           const std::string& drivingState, int urgency);
std::pair<std::string, int> getDrivingState(float distance, float frontSpeed, float egoSpeed);

void getActionAndColor(const std::string& drivingState, float speedChange, std::string& action,
                       cv::Scalar& color);

float updateEgoSpeedSmooth(float currentSpeed, float targetSpeed, int urgencyLevel, float dt);
#endif  // _EGO_VEHICLE_H