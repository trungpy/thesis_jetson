#include <EgoVehicle.h>
using namespace Config;

// --- Function to get driving state ---
std::pair<std::string, int> EgoVehicle::getDrivingState(float distance, float frontSpeed, float egoSpeed) {
    if (distance < config.speedControl.criticalDistance) {
        return {"EMERGENCY_BRAKE", 3};
    } else if (distance < config.speedControl.minFollowingDistance) {
        return {"CLOSE_FOLLOW", 2};
    } else if (distance < config.speedControl.targetFollowingDistance) {
        if (frontSpeed < egoSpeed - 15) {
            return {"SLOW_TRAFFIC", 2};
        } else {
            return {"NORMAL_FOLLOW", 1};
        }
    } else {
        return {"FREE_DRIVE", 0};
    }
}

// --- Function to calculate target speed ---
float EgoVehicle::calculateTargetSpeed(float distance, float frontSpeed, float egoSpeed,
                           const std::string &drivingState, int urgency) {
    if (drivingState == "EMERGENCY_BRAKE") {
        // Emergency: reduce to 70% of current speed immediately
        return std::max(config.speedAdjustment.minSpeedKph, egoSpeed * 0.7f);
    } else if (drivingState == "CLOSE_FOLLOW") {
        // Too close: match front vehicle speed with safety margin
        float safetyMargin = 0.9f;  // 90% of front vehicle speed
        return std::max(config.speedAdjustment.minSpeedKph, frontSpeed * safetyMargin);
    } else if (drivingState == "SLOW_TRAFFIC") {
        // Slow traffic ahead: gradually reduce speed
        float slowTrafficSpeedMargin = 1.1f;
        return std::max(config.speedAdjustment.minSpeedKph,
                        std::min(frontSpeed * slowTrafficSpeedMargin, egoSpeed * 0.95f));
    } else if (drivingState == "NORMAL_FOLLOW") {
        // Normal following: maintain similar speed to front vehicle
        if (std::abs(frontSpeed - egoSpeed) < 5) {
            return egoSpeed;  // Already at good speed
        } else {
            // Gradually converge to front vehicle speed
            return frontSpeed * 0.98f;
        }
    } else {  // FREE_DRIVE
        // No obstacles: can return to cruise speed
        return config.speedControl.cruiseSpeedKph;
    }
}

void EgoVehicle::getActionAndColor(const std::string &drivingState, float speedChange, std::string &action,
                       cv::Scalar &color) {
    if (drivingState == "EMERGENCY_BRAKE") {
        action = "EMERGENCY BRAKE";
        color = cv::Scalar(0, 0, 255);  // Red
    } else if (drivingState == "CLOSE_FOLLOW") {
        action = "BRAKE/SLOW";
        color = cv::Scalar(0, 100, 255);  // Orange-Red
    } else if (drivingState == "SLOW_TRAFFIC") {
        action = "DECELERATE";
        color = cv::Scalar(0, 255, 255);  // Yellow
    } else if (drivingState == "NORMAL_FOLLOW") {
        if (std::abs(speedChange) < 1.0f) {
            action = "MAINTAIN";
            color = cv::Scalar(0, 255, 0);  // Green
        } else if (speedChange > 0) {
            action = "ACCELERATE";
            color = cv::Scalar(255, 255, 0);  // Cyan
        } else {
            action = "DECELERATE";
            color = cv::Scalar(0, 255, 255);  // Yellow
        }
    } else {  // FREE_DRIVE
        if (speedChange > 2.0f) {
            action = "ACCELERATE";
            color = cv::Scalar(255, 255, 0);  // Cyan
        } else {
            action = "CRUISE";
            color = cv::Scalar(0, 255, 0);  // Green
        }
    }
}

float EgoVehicle::updateEgoSpeedSmooth(float currentSpeed, float targetSpeed, int urgencyLevel, float dt) {
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
    float timeFactor =
        std::min(dt / config.speedAdjustment.speedUpdateInterval, 2.0f);  // Cap at 2x
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

void EgoVehicle::updateSpeedControl(double timeStart, int targetId, const cv::Rect &bestBox,
                        float &currentEgoSpeed, double &lastSpeedUpdateTime,
                        std::map<int, std::deque<float>> &objectBuffers,
                        std::map<int, float> &prevDistances, std::map<int, double> &prevTimes,
                        std::map<int, float> &smoothedSpeeds, std::deque<float> &speedChangeHistory,
                        float &avgDistance, float &frontSpeed, std::string &action,
                        cv::Scalar &actionColor) {
    if (targetId != -1 && bestBox.height > 0) {
        float h = bestBox.height;
        float distance = (config.camera.realObjectWidth * config.camera.focalLength) / h;

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
        if (buf.size() > 5) buf.pop_front();

        // ✅ Always update avgDistance
        if (buf.size() >= 3) {
            std::deque<float> sortedBuf = buf;
            std::sort(sortedBuf.begin(), sortedBuf.end());
            avgDistance = sortedBuf[sortedBuf.size() / 2];  // median
        } else {
            avgDistance = std::accumulate(buf.begin(), buf.end(), 0.0f) / buf.size();  // mean
        }

        // ✅ Always update smoothed speed
        double dt = timeStart - prevTimes[targetId];
        if (dt >= config.distanceSpeed.minTimeDelta) {
            float dDist = prevDistances[targetId] - avgDistance;
            if (std::abs(dDist) >= config.distanceSpeed.minDistDelta) {
                float speed = (dDist / dt) * 3.6f;
                smoothedSpeeds[targetId] =
                    config.distanceSpeed.smoothingFactor * speed +
                    (1 - config.distanceSpeed.smoothingFactor) * smoothedSpeeds[targetId];
                prevDistances[targetId] = avgDistance;
                prevTimes[targetId] = timeStart;
            }
        }

        // ✅ Always update front speed every frame
        float relativeSpeed = smoothedSpeeds[targetId];
        frontSpeed = currentEgoSpeed - relativeSpeed;

        // Speed control logic (less frequent)
        if (timeStart - lastSpeedUpdateTime >= config.speedAdjustment.speedUpdateInterval) {
            auto [state, urgency] = getDrivingState(avgDistance, frontSpeed, currentEgoSpeed);
            float targetSpeed =
                calculateTargetSpeed(avgDistance, frontSpeed, currentEgoSpeed, state, urgency);
            float oldSpeed = currentEgoSpeed;

            currentEgoSpeed = updateEgoSpeedSmooth(currentEgoSpeed, targetSpeed, urgency,
                                                   timeStart - lastSpeedUpdateTime);
            float speedDelta = currentEgoSpeed - oldSpeed;

            speedChangeHistory.push_back(speedDelta);
            if (speedChangeHistory.size() > 10) speedChangeHistory.pop_front();

            lastSpeedUpdateTime = timeStart;
            getActionAndColor(state, speedDelta, action, actionColor);

            std::cout << "[+] ID " << targetId << " | Dist: " << std::fixed << std::setprecision(1)
                      << avgDistance << "m | Front: " << frontSpeed
                      << " km/h | Ego: " << currentEgoSpeed << " km/h | State: " << state
                      << " | Action: " << action << std::endl;
        }
    } else {
        // No target: cruise mode
        if (timeStart - lastSpeedUpdateTime >= config.speedAdjustment.speedUpdateInterval) {
            if (std::abs(currentEgoSpeed - config.speedControl.cruiseSpeedKph) > 1) {
                currentEgoSpeed += (currentEgoSpeed < config.speedControl.cruiseSpeedKph)
                                       ? config.speedAdjustment.gentleAdjustment
                                       : -config.speedAdjustment.gentleAdjustment;
                currentEgoSpeed =
                    std::clamp(currentEgoSpeed, 0.0f, config.distanceSpeed.maxValidSpeedKph);
            }
            lastSpeedUpdateTime = timeStart;
        }

        // ❗ Reset or mark frontSpeed and avgDistance as unavailable
        frontSpeed = 0.0f;
        avgDistance = -1.0f;
        action = "Deactivated";
        actionColor = cv::Scalar(200, 200, 200);
    }
}