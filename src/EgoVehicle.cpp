#include <EgoVehicle.h>

#include <algorithm>
using namespace Config;
// --- Function to get driving state ---
std::pair<DrivingState, int>
EgoVehicle::getDrivingState(float distance, float frontSpeed, float egoSpeed) {
    const float speedDiff = egoSpeed - frontSpeed;
    const float closeGap = config.speedControl.minFollowingDistance;
    const float safeGap = config.speedControl.targetFollowingDistance;
    const float criticalGap = config.speedControl.criticalDistance;

    if (distance < criticalGap) {
        return {DrivingState::emergencyBrake, 3};
    } else if (distance < closeGap) {
        return {DrivingState::closeFollow, 2};
    } else if (distance < safeGap) {
        if (speedDiff > 10) {
            return {DrivingState::slowTraffic, 2};
        } else {
            return {DrivingState::normalFollow, 1};
        }
    } else {
        if (speedDiff > 20) {
            return {DrivingState::slowTraffic, 1};
        }
        return {DrivingState::freeDrive, 0};
    }
}

// --- Function to calculate target speed ---
float EgoVehicle::calculateTargetSpeed(float distance, float frontSpeed,
                                       float egoSpeed,
                                       DrivingState drivingState, int urgency) {
    const float minSpeed = config.speedAdjustment.minSpeedKph;
    const float maxSpeed = config.speedAdjustment.maxSpeedKph;

    switch (drivingState) {
    case DrivingState::emergencyBrake:
        return std::max(minSpeed, egoSpeed * 0.5f);
    case DrivingState::closeFollow:
        return std::clamp(frontSpeed * 0.9f, minSpeed, egoSpeed);
    case DrivingState::slowTraffic:
        return std::clamp(std::min(frontSpeed * 1.05f, egoSpeed * 0.95f),
                          minSpeed, egoSpeed);
    case DrivingState::normalFollow:
        if (std::abs(frontSpeed - egoSpeed) < 3.0f)
            return egoSpeed;
        return std::clamp(frontSpeed, minSpeed, maxSpeed);
    case DrivingState::freeDrive:
    default:
        return config.speedControl.cruiseSpeedKph;
    }
}
void EgoVehicle::getActionAndColor(DrivingState drivingState, float speedChange,
                                   float egoSpeed, std::string &action,
                                   cv::Scalar &color) {
    this->throttleCmd = 0.0f;
    this->brakeCmd = 0.0f;

    switch (drivingState) {
    case DrivingState::emergencyBrake:
        action = "EmergencyBrake";
        color = cv::Scalar(0, 0, 255);
        this->brakeCmd = 1.0f; // Full brake
        break;

    case DrivingState::closeFollow:
        action = "BrakeSlow";
        color = cv::Scalar(0, 100, 255);
        this->brakeCmd =
            std::clamp(std::abs(speedChange) * 0.15f, 0.4f, 0.8f); // adaptive
        break;

    case DrivingState::slowTraffic:
        action = "Decelerate";
        color = cv::Scalar(0, 255, 255);
        this->brakeCmd =
            std::clamp(std::abs(speedChange) * 0.1f, 0.2f, 0.5f); // smooth
        break;

    case DrivingState::normalFollow:
        if (std::abs(speedChange) < 0.5f) {
            action = "Maintain";
            color = cv::Scalar(0, 255, 0);
            this->throttleCmd = std::clamp(0.1f + 0.005f * egoSpeed, 0.1f,
                                           0.25f); // adaptive base throttle
        } else if (speedChange > 0) {
            action = "Accelerate";
            color = cv::Scalar(255, 255, 0);
            this->throttleCmd =
                std::clamp(speedChange * 0.08f, 0.2f, 0.5f); // smoother gain
        } else {
            action = "Decelerate";
            color = cv::Scalar(0, 255, 255);
            this->brakeCmd = std::clamp(std::abs(speedChange) * 0.12f, 0.1f,
                                        0.5f); // dynamic
        }
        break;

    case DrivingState::freeDrive: {
        const float cruiseSpeed = config.speedControl.cruiseSpeedKph;
        if (egoSpeed >= cruiseSpeed - 0.5f) {
            action = "CruiseCoast";
            color = cv::Scalar(0, 200, 0);
        } else if (speedChange > 1.5f) {
            action = "Accelerate";
            color = cv::Scalar(255, 255, 0);
            this->throttleCmd = std::clamp(speedChange * 0.1f, 0.3f, 0.7f);
        } else {
            action = "Cruise";
            color = cv::Scalar(0, 255, 0);
            this->throttleCmd = std::clamp(0.1f + 0.005f * egoSpeed, 0.1f,
                                           0.35f); // soft cruise
        }
        break;
    }
    }
}

float EgoVehicle::updateEgoSpeedSmooth(float currentSpeed, float targetSpeed,
                                       int urgencyLevel, float dt) {
    float speedDiff = targetSpeed - currentSpeed;

    // Determine adjustment rate
    float maxChange;
    if (urgencyLevel >= 3) {
        maxChange = config.speedAdjustment.aggressiveAdjustment;
    } else if (urgencyLevel >= 2) {
        maxChange = config.speedAdjustment.moderateAdjustment;
    } else if (urgencyLevel >= 1) {
        maxChange = config.speedAdjustment.gentleAdjustment;
    } else {
        maxChange = config.speedAdjustment.gentleAdjustment * 0.5f;
    }

    // Time-based scaling
    float timeFactor = std::min(dt / config.speedAdjustment.speedUpdateInterval,
                                2.0f); // Cap at 2x
    maxChange *= timeFactor;

    float newSpeed;
    if (std::abs(speedDiff) <= maxChange) {
        newSpeed = targetSpeed;
    } else if (speedDiff > 0) {
        newSpeed = currentSpeed + maxChange;
    } else {
        newSpeed = currentSpeed - maxChange;
    }

    // Clamp to min/max speed
    newSpeed = std::clamp(newSpeed, config.speedAdjustment.minSpeedKph,
                          config.speedAdjustment.maxSpeedKph);

    return newSpeed;
}

void EgoVehicle::updateSpeedControl(
    double timeStart, int targetId, const cv::Rect &bestBox,
    float &currentEgoSpeed, double &lastSpeedUpdateTime,
    std::map<int, std::deque<float>> &objectBuffers,
    std::map<int, float> &prevDistances, std::map<int, double> &prevTimes,
    std::map<int, float> &smoothedSpeeds, std::deque<float> &speedChangeHistory,
    float &avgDistance, float &frontSpeed, std::string &action,
    cv::Scalar &actionColor) {
    if (targetId != -1 && bestBox.height > 0) {
        float h = bestBox.height;
        float distance =
            (config.camera.realObjectWidth * config.camera.focalLength) / h;

        // Initialize if new
        if (objectBuffers.find(targetId) == objectBuffers.end()) {
            objectBuffers[targetId] = std::deque<float>();
            prevDistances[targetId] = distance;
            prevTimes[targetId] = timeStart;
            smoothedSpeeds[targetId] = 0.0f;
        }

        // Push to buffer
        auto &buf = objectBuffers[targetId];
        buf.push_back(distance);
        if (buf.size() > 5)
            buf.pop_front();

        // ✅ Always update avgDistance
        if (buf.size() >= 3) {
            std::deque<float> sortedBuf = buf;
            std::sort(sortedBuf.begin(), sortedBuf.end());
            avgDistance = sortedBuf[sortedBuf.size() / 2]; // median
        } else {
            avgDistance = std::accumulate(buf.begin(), buf.end(), 0.0f) /
                          buf.size(); // mean
        }
        // ✅ Always update smoothed speed
        double dt = timeStart - prevTimes[targetId];
        if (dt >= config.distanceSpeed.minTimeDelta) {
            float dDist = prevDistances[targetId] - avgDistance;
            if (std::abs(dDist) >= config.distanceSpeed.minDistDelta) {
                float speed = (dDist / dt) * 3.6f;
                smoothedSpeeds[targetId] =
                    config.distanceSpeed.smoothingFactor * speed +
                    (1 - config.distanceSpeed.smoothingFactor) *
                        smoothedSpeeds[targetId];
                prevDistances[targetId] = avgDistance;
                prevTimes[targetId] = timeStart;
            }
        }

        // ✅ Always update front speed every frame
        float relativeSpeed = smoothedSpeeds[targetId];
        frontSpeed = currentEgoSpeed - relativeSpeed;

        // Speed control logic (less frequent)
        if (timeStart - lastSpeedUpdateTime >=
            config.speedAdjustment.speedUpdateInterval) {
            auto [state, urgency] =
                this->getDrivingState(avgDistance, frontSpeed, currentEgoSpeed);
            float targetSpeed = this->calculateTargetSpeed(
                avgDistance, frontSpeed, currentEgoSpeed, state, urgency);
            float oldSpeed = currentEgoSpeed;

            currentEgoSpeed = this->updateEgoSpeedSmooth(
                currentEgoSpeed, targetSpeed, urgency,
                timeStart - lastSpeedUpdateTime);
            float speedDelta = currentEgoSpeed - oldSpeed;

            speedChangeHistory.push_back(speedDelta);
            if (speedChangeHistory.size() > 10)
                speedChangeHistory.pop_front();

            lastSpeedUpdateTime = timeStart;
            this->getActionAndColor(state, speedDelta, currentEgoSpeed, action,
                                    actionColor);
        }
    } else {
        currentEgoSpeed = 70.0f;
        // No target: cruise mode
        if (timeStart - lastSpeedUpdateTime >=
            config.speedAdjustment.speedUpdateInterval) {
            if (std::abs(currentEgoSpeed - config.speedControl.cruiseSpeedKph) >
                1) {
                currentEgoSpeed +=
                    (currentEgoSpeed < config.speedControl.cruiseSpeedKph)
                        ? config.speedAdjustment.gentleAdjustment
                        : -config.speedAdjustment.gentleAdjustment;
                currentEgoSpeed =
                    std::clamp(currentEgoSpeed, 0.0f,
                               config.distanceSpeed.maxValidSpeedKph);
            }
            lastSpeedUpdateTime = timeStart;
        }

        // ❗ Reset or mark frontSpeed and avgDistance as unavailable
        frontSpeed = 0.0f;
        avgDistance = -1.0f;
        action = "Deactivated";
        this->throttleCmd = 0.0;
        this->brakeCmd = 0.0;
        actionColor = cv::Scalar(200, 200, 200);
    }
}

std::string toString(DrivingState state) {
    switch (state) {
    case DrivingState::emergencyBrake:
        return "emergencyBrake";
    case DrivingState::closeFollow:
        return "closeFollow";
    case DrivingState::slowTraffic:
        return "slowTraffic";
    case DrivingState::normalFollow:
        return "normalFollow";
    case DrivingState::freeDrive:
        return "freeDrive";
    default:
        return "unknown";
    }
}
