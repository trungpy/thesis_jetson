#include "config.h"

#include <fstream>
#include <stdexcept>

namespace Config {

Configuration config;
// HUD colors
const cv::Scalar white(255, 255, 255);
const cv::Scalar red(68, 68, 255);
const cv::Scalar yellow(0, 255, 255);
const cv::Scalar orange(0, 165, 255);
const cv::Scalar green(102, 255, 102);
const cv::Scalar gray(180, 180, 180);

const std::vector<std::vector<unsigned int>> colors = {
    {220, 20, 60},  {119, 172, 48}, {0, 114, 189},   {237, 177, 32},  {126, 47, 142},
    {217, 83, 25},  {255, 0, 0},    {153, 153, 153}, {255, 255, 255}, {255, 0, 0},
    {255, 255, 0},  {0, 255, 0},    {170, 255, 0},   {200, 255, 0},   {255, 255, 0},
    {255, 200, 0},  {255, 170, 0},  {255, 85, 0},    {180, 180, 180}, {140, 140, 140},
    {100, 100, 100}};

void loadConfig(const std::string& configPath) {
    try {
        std::ifstream configFile(configPath);
        if (!configFile.is_open()) {
            throw std::runtime_error("Could not open config file: " + configPath);
        }

        nlohmann::json jsonConfig;
        configFile >> jsonConfig;

        // Parse camera settings
        config.camera = {jsonConfig["camera"]["width"], jsonConfig["camera"]["height"],
                         jsonConfig["camera"]["fps"], jsonConfig["camera"]["focalLength"],
                         jsonConfig["camera"]["realObjectWidth"]};

        // Parse ROI
        config.roi = {jsonConfig["roi"]["xMin"], jsonConfig["roi"]["xMax"]};
        config.detectConfig = {jsonConfig["detectConfig"]["confThreshold"],
                               jsonConfig["detectConfig"]["nmsThreshold"]};
        // Parse distance and speed estimation
        config.distanceSpeed = {jsonConfig["distanceSpeedEstimation"]["maxValidSpeedKph"],
                                jsonConfig["distanceSpeedEstimation"]["minDistDelta"],
                                jsonConfig["distanceSpeedEstimation"]["smoothingFactor"],
                                jsonConfig["distanceSpeedEstimation"]["minTimeDelta"],
                                jsonConfig["distanceSpeedEstimation"]["minSpeedThreshold"]};

        // Parse adaptive speed control
        config.speedControl = {jsonConfig["adaptiveSpeedControl"]["initialSpeedKph"],
                               jsonConfig["adaptiveSpeedControl"]["cruiseSpeedKph"],
                               jsonConfig["adaptiveSpeedControl"]["targetFollowingDistance"],
                               jsonConfig["adaptiveSpeedControl"]["minFollowingDistance"],
                               jsonConfig["adaptiveSpeedControl"]["criticalDistance"]};

        // Parse speed adjustment
        config.speedAdjustment = {jsonConfig["speedAdjustment"]["speedUpdateInterval"],
                                  jsonConfig["speedAdjustment"]["gentleAdjustment"],
                                  jsonConfig["speedAdjustment"]["moderateAdjustment"],
                                  jsonConfig["speedAdjustment"]["aggressiveAdjustment"],
                                  jsonConfig["speedAdjustment"]["minSpeedKph"],
                                  jsonConfig["speedAdjustment"]["maxSpeedKph"]};

        // Parse object tracking
        config.objectTracking.trackClasses =
            jsonConfig["objectTracking"]["trackClasses"].get<std::vector<int>>();
        config.objectTracking.classNames =
            jsonConfig["objectTracking"]["classNames"].get<std::vector<std::string>>();
        config.objectTracking.colors = colors;  // Use predefined colors
    } catch (const std::exception& e) {
        throw std::runtime_error("Failed to parse config file: " + std::string(e.what()));
    }
}

}  // namespace Config