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
    // Constructor
    EgoVehicle()
        : throttleCmd(0.0f), brakeCmd(0.0f), currentAcceleration(0.0f) {}

    void updateSpeedControl(double timeStart, int targetId,
                            const cv::Rect &bestBox, float &currentEgoSpeed,
                            double &lastSpeedUpdateTime,
                            std::map<int, std::deque<float>> &objectBuffers,
                            std::map<int, float> &prevDistances,
                            std::map<int, double> &prevTimes,
                            std::map<int, float> &smoothedSpeeds,
                            std::deque<float> &speedChangeHistory,
                            float &avgDistance, float &frontSpeed,
                            cv::Scalar &actionColor);

    float getThrottleCmd() { return this->throttleCmd; }
    float getBrakeCmd() { return this->brakeCmd; }
    float getCurrentAcceleration() { return this->currentAcceleration; }
    std::string getAction() { return this->action; }

private:
    std::string action;
    float throttleCmd;
    float brakeCmd;
    float currentAcceleration; // Current acceleration in m/s²

    // Modified methods for acceleration-based control
    float updateEgoSpeedWithAcceleration(float currentSpeed,
                                         float targetAcceleration,
                                         int urgencyLevel, float dt);
    void getActionAndColor(DrivingState drivingState, float acceleration,
                           float egoSpeed, cv::Scalar &color);

    // New method for calculating target acceleration
    float calculateTargetAcceleration(float distance, float frontSpeed,
                                      float egoSpeed, DrivingState drivingState,
                                      int urgency);

    // Existing methods
    std::pair<DrivingState, int>
    getDrivingState(float distance, float frontSpeed, float egoSpeed);
};

#endif // _EGO_VEHICLE_H