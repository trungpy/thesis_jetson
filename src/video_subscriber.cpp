#include "modules/EnhancedVideoSubscriber.h"
#include <app.h>
#include <ros/ros.h>

using namespace Config;

int main(int argc, char **argv) {
    ros::init(argc, argv, "enhanced_video_subscriber");

    try {
        ROS_INFO("Starting Enhanced Video Subscriber...");
        ROS_INFO("Features: Enhanced detection, lane tracking, autonomous "
                 "control, data logging");

        std::cout << "🔧 Loading configuration..." << std::endl;
        try {
            Config::loadConfig("/home/trung/catkin_ws/src/thesis/config.json");
        } catch (const std::exception &e) {
            ROS_ERROR("Failed to load config.json: %s", e.what());
            return -1;
        }

        CameraSettings cameraSettings = Config::config.camera;
        std::cout << "📷 Camera settings loaded:\n";
        std::cout << "   Focal Length: " << cameraSettings.focalLength
                  << " mm\n";
        std::cout << "   Real Object Width: " << cameraSettings.realObjectWidth
                  << " m\n";
        std::cout << "   FPS: " << cameraSettings.fps << " FPS\n";
        std::cout << "✅ Configuration loaded successfully.\n";

        try {
            STrack::initializeEstimator();
            ROS_INFO("✅ Tracking estimator initialized successfully");
        } catch (const std::exception &e) {
            ROS_ERROR("Failed to initialize tracking estimator: %s", e.what());
            return -1;
        }

        EnhancedVideoSubscriber subscriber;

        ROS_INFO("🚗 Enhanced Video Subscriber is ready!");
        ROS_INFO("📡 Subscribed to: /video/image");
        ROS_INFO("📊 Publishing topics:");
        ROS_INFO("   - /ego_speed (Float32)");
        ROS_INFO("   - /driving_action (String)");
        ROS_INFO("   - /control/throttle (Float32)");
        ROS_INFO("   - /control/brake (Float32)");

        ros::spin();

    } catch (const std::exception &e) {
        ROS_ERROR("Exception in main: %s", e.what());
        return -1;
    }

    ROS_INFO("Enhanced Video Subscriber shutting down...");
    return 0;
}