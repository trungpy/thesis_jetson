// config.h - Centralized configuration for camera, detection, and control parameters.
//
// - Camera & Frame Settings: Resolution, fps, and camera calibration (focal length, object width).
// - ROI: X-axis bounds for focusing detection/tracking.
// - Distance & Speed Estimation: Limits and smoothing for object speed/distance calculations.
// - Adaptive Speed Control: Desired speeds, following distances, and emergency thresholds.
// - Speed Adjustment: How quickly speed can change, and min/max allowed speeds.
// - Object Tracking: Which classes to track, their names, and display colors.
//
// All values are defined as constexpr or const for compile-time safety and easy tuning.
//
// Usage: Include this header wherever these parameters are needed.

#ifndef CONFIG_H
#define CONFIG_H

#include <nlohmann/json.hpp>
#include <opencv2/core.hpp>
#include <string>
#include <vector>

namespace Config {

// Structs to organize configuration data
struct CameraSettings {
    int width;
    int height;
    int fps;
    float focalLength;
    float realObjectWidth;
};

struct ROI {
    float xMin;
    float xMax;
};

struct DistanceSpeedEstimation {
    float maxValidSpeedKph;
    float minDistDelta;
    float smoothingFactor;
    float minTimeDelta;
    float minSpeedThreshold;
};

struct AdaptiveSpeedControl {
    float initialSpeedKph;
    int cruiseSpeedKph;
    float targetFollowingDistance;
    float minFollowingDistance;
    float criticalDistance;
};

struct SpeedAdjustment {
    float speedUpdateInterval;
    float gentleAdjustment;
    float moderateAdjustment;
    float aggressiveAdjustment;
    float minSpeedKph;
    float maxSpeedKph;
};

struct ObjectTracking {
    std::vector<int> trackClasses;
    std::vector<std::string> classNames;
    std::vector<std::vector<unsigned int>> colors;  // RGB colors for each class
};

struct DetectConfig {
    // Confidence threshold for filtering detections
    float confThreshold;

    // Non-Maximum Suppression (NMS) threshold to remove duplicate boxes
    float nmsThreshold;
};

// Main configuration structure
struct Configuration {
    DetectConfig detectConfig;  // Detection configuration
    CameraSettings camera;
    ROI roi;
    DistanceSpeedEstimation distanceSpeed;
    AdaptiveSpeedControl speedControl;
    SpeedAdjustment speedAdjustment;
    ObjectTracking objectTracking;
};

// Global configuration instance
extern Configuration config;

// HUD colors (not loaded from JSON)
extern const cv::Scalar white;
extern const cv::Scalar red;
extern const cv::Scalar yellow;
extern const cv::Scalar orange;
extern const cv::Scalar green;
extern const cv::Scalar gray;

// Initialization function
void loadConfig(const std::string& configPath);

}  // namespace Config

#endif  // CONFIG_H