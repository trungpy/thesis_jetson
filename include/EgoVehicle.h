#ifndef _EGO_VEHICLE_H
#define _EGO_VEHICLE_H

#include <config.h>

#include <cmath>
#include <deque>
#include <iostream>
#include <map>
#include <opencv2/core.hpp>
#include <string>
#include <tuple>

class EgoVehicle {
   public:
    static void updateSpeedControl(double timeStart, int targetId, const cv::Rect &bestBox,
                                   float &currentEgoSpeed, double &lastSpeedUpdateTime,
                                   std::map<int, std::deque<float>> &objectBuffers,
                                   std::map<int, float> &prevDistances,
                                   std::map<int, double> &prevTimes,
                                   std::map<int, float> &smoothedSpeeds,
                                   std::deque<float> &speedChangeHistory, float &avgDistance,
                                   float &frontSpeed, std::string &action, cv::Scalar &actionColor);

   private:
    static float updateEgoSpeedSmooth(float currentSpeed, float targetSpeed, int urgencyLevel,
                                      float dt);
    static void getActionAndColor(const std::string &drivingState, float speedChange,
                                  std::string &action, cv::Scalar &color);
    static std::pair<std::string, int> getDrivingState(float distance, float frontSpeed,
                                                       float egoSpeed);
    static float calculateTargetSpeed(float distance, float frontSpeed, float egoSpeed,
                                      const std::string &drivingState, int urgency);
};

#endif  // _EGO_VEHICLE_H
