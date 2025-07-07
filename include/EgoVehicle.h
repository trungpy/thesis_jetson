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
enum class DrivingState {
    emergencyBrake,
    closeFollow,
    slowTraffic,
    normalFollow,
    freeDrive
};

class EgoVehicle {
public:
    void updateSpeedControl(double timeStart, int targetId,
                            const cv::Rect &bestBox, float &currentEgoSpeed,
                            double &lastSpeedUpdateTime,
                            std::map<int, std::deque<float>> &objectBuffers,
                            std::map<int, float> &prevDistances,
                            std::map<int, double> &prevTimes,
                            std::map<int, float> &smoothedSpeeds,
                            std::deque<float> &speedChangeHistory,
                            float &avgDistance, float &frontSpeed,
                            std::string &action, cv::Scalar &actionColor);

    float getThrottleCmd() { return this->throttleCmd; }
    float getBrakeCmd() { return this->brakeCmd; }

private:
    float throttleCmd;
    float brakeCmd;
    float updateEgoSpeedSmooth(float currentSpeed, float targetSpeed,
                               int urgencyLevel, float dt);
    void getActionAndColor(DrivingState drivingState, float speedChange,
                           float egoSpeed, std::string &action,
                           cv::Scalar &color);
    std::pair<DrivingState, int>
    getDrivingState(float distance, float frontSpeed, float egoSpeed);
    float calculateTargetSpeed(float distance, float frontSpeed, float egoSpeed,
                               DrivingState drivingState, int urgency);
};

#endif // _EGO_VEHICLE_H
