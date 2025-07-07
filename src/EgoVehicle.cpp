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

// --- Modified function to get this->action and color ---
void EgoVehicle::getActionAndColor(DrivingState drivingState,
                                   float acceleration, float egoSpeed,
                                   cv::Scalar &color) {
    // Convert acceleration to throttle/brake commands
    if (acceleration > 0.1f) {
        // Positive acceleration -> throttle
        this->throttleCmd = std::clamp(
            acceleration / config.speedAdjustment.maxAcceleration, 0.1f, 1.0f);
        this->brakeCmd = 0.0f;
    } else if (acceleration < -0.1f) {
        // Negative acceleration -> brake
        this->brakeCmd = std::clamp(
            -acceleration / config.speedAdjustment.maxDeceleration, 0.1f, 1.0f);
        this->throttleCmd = 0.0f;
    } else {
        // Near zero acceleration -> coast
        this->throttleCmd = 0.0f;
        this->brakeCmd = 0.0f;
    }

    // Set this->action and color based on driving state and acceleration
    switch (drivingState) {
    case DrivingState::emergencyBrake:
        this->action = "EmergencyBrake";
        color = cv::Scalar(0, 0, 255);
        break;

    case DrivingState::closeFollow:
        this->action = "BrakeSlow";
        color = cv::Scalar(0, 100, 255);
        break;

    case DrivingState::slowTraffic:
        this->action = "Decelerate";
        color = cv::Scalar(0, 255, 255);
        break;

    case DrivingState::normalFollow:
        if (std::abs(acceleration) < 0.1f) {
            this->action = "Maintain";
            color = cv::Scalar(0, 255, 0);
        } else if (acceleration > 0) {
            this->action = "Accelerate";
            color = cv::Scalar(255, 255, 0);
        } else {
            this->action = "Decelerate";
            color = cv::Scalar(0, 255, 255);
        }
        break;

    case DrivingState::freeDrive:
        if (acceleration > 0.2f) {
            this->action = "Accelerate";
            color = cv::Scalar(255, 255, 0);
        } else if (acceleration < -0.1f) {
            this->action = "Decelerate";
            color = cv::Scalar(0, 255, 255);
        } else {
            this->action = "Cruise";
            color = cv::Scalar(0, 255, 0);
        }
        break;
    }
}

// --- NEW: Function to calculate target acceleration ---
float EgoVehicle::calculateTargetAcceleration(float distance, float frontSpeed,
                                              float egoSpeed,
                                              DrivingState drivingState,
                                              int urgency) {
    const float speedDiff = egoSpeed - frontSpeed;
    const float maxAccel =
        config.speedAdjustment.maxAcceleration; // e.g., 3.0 m/s²
    const float maxDecel =
        config.speedAdjustment.maxDeceleration; // e.g., -5.0 m/s²
    const float comfortDecel =
        config.speedAdjustment.comfortDeceleration; // e.g., -2.0 m/s²

    switch (drivingState) {
    case DrivingState::emergencyBrake:
        return maxDecel; // Maximum deceleration

    case DrivingState::closeFollow: {
        // Aggressive deceleration proportional to speed difference
        float decelFactor = std::clamp(speedDiff / 20.0f, 0.3f, 1.0f);
        return maxDecel * decelFactor;
    }

    case DrivingState::slowTraffic: {
        // Moderate deceleration
        float decelFactor = std::clamp(speedDiff / 15.0f, 0.2f, 0.7f);
        return comfortDecel * decelFactor;
    }

    case DrivingState::normalFollow: {
        // Proportional control based on speed difference
        if (std::abs(speedDiff) < 3.0f) {
            return 0.0f; // Maintain current speed
        }

        float targetAccel = -speedDiff * 0.3f; // Proportional gain
        return std::clamp(targetAccel, maxDecel, maxAccel);
    }

    case DrivingState::freeDrive: {
        // Regulate speed towards cruise speed (both accelerate and decelerate)
        float cruiseSpeed =
            config.speedControl.cruiseSpeedKph / 3.6f; // Convert to m/s
        float currentSpeedMS = egoSpeed / 3.6f; // Convert current speed to m/s
        float speedError = cruiseSpeed - currentSpeedMS;

        if (std::abs(speedError) < 0.5f) {
            return 0.0f; // Close enough to cruise speed
        }

        // Proportional control: positive error = accelerate, negative error =
        // decelerate
        float targetAccel = speedError * 0.5f; // Proportional gain

        // Allow full deceleration range when over cruise speed
        if (speedError < 0) { // Over cruise speed, need to decelerate
            return std::clamp(targetAccel, maxDecel * 0.5f, 0.0f);
        } else { // Under cruise speed, need to accelerate
            return std::clamp(targetAccel, 0.0f, maxAccel * 0.7f);
        }
    }

    default:
        return 0.0f;
    }
}

float EgoVehicle::updateEgoSpeedWithAcceleration(float currentSpeed,
                                                 float targetAcceleration,
                                                 int urgencyLevel, float dt) {
    // Convert speed from km/h to m/s for physics calculations
    float currentSpeedMS = currentSpeed / 3.6f;

    // Apply acceleration limits based on urgency
    float maxAccelLimit = config.speedAdjustment.maxAcceleration;
    float maxDecelLimit = config.speedAdjustment.maxDeceleration;

    if (urgencyLevel >= 3) {
        // Emergency: use full acceleration limits
        // Already at max values
    } else if (urgencyLevel >= 2) {
        // Moderate: reduce limits slightly
        maxAccelLimit *= 0.8f;
        maxDecelLimit *= 0.8f;
    } else if (urgencyLevel >= 1) {
        // Gentle: further reduce limits
        maxAccelLimit *= 0.6f;
        maxDecelLimit *= 0.6f;
    } else {
        // Very gentle: minimal acceleration
        maxAccelLimit *= 0.4f;
        maxDecelLimit *= 0.4f;
    }

    // Clamp the target acceleration
    float clampedAcceleration =
        std::clamp(targetAcceleration, maxDecelLimit, maxAccelLimit);

    // Apply jerk limiting (smooth acceleration changes)
    float maxJerk = config.speedAdjustment.maxJerk; // e.g., 2.0 m/s³
    float accelChange = clampedAcceleration - this->currentAcceleration;
    float maxAccelChange = maxJerk * dt;

    if (std::abs(accelChange) > maxAccelChange) {
        this->currentAcceleration +=
            (accelChange > 0) ? maxAccelChange : -maxAccelChange;
    } else {
        this->currentAcceleration = clampedAcceleration;
    }

    // Update speed using physics: v = v₀ + a*t
    float newSpeedMS = currentSpeedMS + this->currentAcceleration * dt;

    // Convert back to km/h and clamp to limits
    float newSpeedKMH = newSpeedMS * 3.6f;
    newSpeedKMH = std::clamp(newSpeedKMH, config.speedAdjustment.minSpeedKph,
                             config.speedAdjustment.maxSpeedKph);

    return newSpeedKMH;
}
// --- Modified main update function ---
void EgoVehicle::updateSpeedControl(
    double timeStart, int targetId, const cv::Rect &bestBox,
    float &currentEgoSpeed, double &lastSpeedUpdateTime,
    std::map<int, std::deque<float>> &objectBuffers,
    std::map<int, float> &prevDistances, std::map<int, double> &prevTimes,
    std::map<int, float> &smoothedSpeeds, std::deque<float> &speedChangeHistory,
    float &avgDistance, float &frontSpeed, cv::Scalar &actionColor) {

    float dt = timeStart - lastSpeedUpdateTime;

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

        // Push to buffer and calculate average distance
        auto &buf = objectBuffers[targetId];
        buf.push_back(distance);
        if (buf.size() > 5)
            buf.pop_front();

        if (buf.size() >= 3) {
            std::deque<float> sortedBuf = buf;
            std::sort(sortedBuf.begin(), sortedBuf.end());
            avgDistance = sortedBuf[sortedBuf.size() / 2]; // median
        } else {
            avgDistance = std::accumulate(buf.begin(), buf.end(), 0.0f) /
                          buf.size(); // mean
        }

        // Calculate front vehicle speed
        double timeDiff = timeStart - prevTimes[targetId];
        if (timeDiff >= config.distanceSpeed.minTimeDelta) {
            float dDist = prevDistances[targetId] - avgDistance;
            if (std::abs(dDist) >= config.distanceSpeed.minDistDelta) {
                float speed = (dDist / timeDiff) * 3.6f;
                smoothedSpeeds[targetId] =
                    config.distanceSpeed.smoothingFactor * speed +
                    (1 - config.distanceSpeed.smoothingFactor) *
                        smoothedSpeeds[targetId];
                prevDistances[targetId] = avgDistance;
                prevTimes[targetId] = timeStart;
            }
        }

        float relativeSpeed = smoothedSpeeds[targetId];
        frontSpeed = currentEgoSpeed - relativeSpeed;

        // Speed control using acceleration
        if (dt >= config.speedAdjustment.speedUpdateInterval) {
            auto [state, urgency] =
                this->getDrivingState(avgDistance, frontSpeed, currentEgoSpeed);

            float targetAcceleration = this->calculateTargetAcceleration(
                avgDistance, frontSpeed, currentEgoSpeed, state, urgency);

            currentEgoSpeed = this->updateEgoSpeedWithAcceleration(
                currentEgoSpeed, targetAcceleration, urgency, dt);

            // Clamp ego speed to not exceed cruiseSpeedKph
            float cruiseSpeed = config.speedControl.cruiseSpeedKph;
            if (currentEgoSpeed > cruiseSpeed) {
                currentEgoSpeed = cruiseSpeed;
            }

            // Store acceleration for history tracking
            speedChangeHistory.push_back(this->currentAcceleration);
            if (speedChangeHistory.size() > 10)
                speedChangeHistory.pop_front();

            lastSpeedUpdateTime = timeStart;
            this->getActionAndColor(state, this->currentAcceleration,
                                    currentEgoSpeed, actionColor);
            // Calculate engine, throttle, and brake forces
            this->calculateEngineForces(currentEgoSpeed);
        }
    } else {
        // No target: cruise mode
        if (dt >= config.speedAdjustment.speedUpdateInterval) {
            float cruiseSpeed = config.speedControl.cruiseSpeedKph;
            float speedError = cruiseSpeed - currentEgoSpeed;

            if (std::abs(speedError) > 1.0f) {
                float targetAcceleration =
                    speedError * 0.1f; // Gentle proportional control
                currentEgoSpeed = this->updateEgoSpeedWithAcceleration(
                    currentEgoSpeed, targetAcceleration, 0, dt);
            }

            // Clamp ego speed to not exceed cruiseSpeedKph
            float maxCruiseSpeed = config.speedControl.cruiseSpeedKph;
            if (currentEgoSpeed > maxCruiseSpeed) {
                currentEgoSpeed = maxCruiseSpeed;
            }

            lastSpeedUpdateTime = timeStart;
            // Calculate engine, throttle, and brake forces
            this->calculateEngineForces(currentEgoSpeed);
        }

        frontSpeed = 0.0f;
        avgDistance = -1.0f;
        this->action = "Deactivated";
        this->throttleCmd = 0.0;
        this->brakeCmd = 0.0;
        actionColor = cv::Scalar(200, 200, 200);
    }
}

// --- NEW: Function to calculate engine, throttle, and brake forces ---
void EgoVehicle::calculateEngineForces(float egoSpeed) {
    // Assume vehicle mass (kg)
    const float vehicleMass = 1500.0f;
    // Convert speed from km/h to m/s
    float speedMS = egoSpeed / 3.6f;
    // Engine force = mass * acceleration
    this->engineForce = vehicleMass * this->currentAcceleration;
    // Throttle force = proportional to throttleCmd
    this->throttleForce = vehicleMass * this->throttleCmd * config.speedAdjustment.maxAcceleration;
    // Brake force = proportional to brakeCmd
    this->brakeForce = vehicleMass * this->brakeCmd * (-config.speedAdjustment.maxDeceleration);
}
