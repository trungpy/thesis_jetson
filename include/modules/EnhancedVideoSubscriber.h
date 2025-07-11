#ifndef ENHANCED_VIDEO_SUBSCRIBER_H
#define ENHANCED_VIDEO_SUBSCRIBER_H

#include <chrono>
#include <cv_bridge/cv_bridge.h>
#include <deque>
#include <geometry_msgs/Twist.h>
#include <image_transport/image_transport.h>
#include <map>
#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>

#include "DataLogger.h"
#include "EgoVehicle.h"
#include "HUDRenderer.h"
#include "TrafficSignStabilizer.hpp"
#include "VideoRecorder.h"
#include <BYTETracker.h>
#include <Detect.h>
#include <LaneDetector.h>
#include <Logger.h>
#include <utils.hpp>

class EnhancedVideoSubscriber {
private:
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    ros::Publisher speed_pub_;
    ros::Publisher action_pub_;
    ros::Publisher throttle_pub_;
    ros::Publisher brake_pub_;

    std::string model_path_;
    std::string output_dir_;
    bool enable_debug_output_;
    bool enable_data_logging_;
    bool enable_data_writer_;
    Detect model_;
    LaneDetector laneDetector_;
    BYTETracker tracker_;
    int frameCount_;
    std::chrono::steady_clock::time_point fpsStartTime_;
    double fps_;

    int maxSpeed_;
    int accSpeed_;
    float currentEgoSpeed_;
    double lastSpeedUpdateTime_;
    std::deque<float> speedChangeHistory_;
    std::deque<float> distanceHistory_;

    std::map<int, std::deque<float>> objectBuffers_;
    std::map<int, float> prevDistances_;
    std::map<int, double> prevTimes_;
    std::map<int, float> smoothedSpeeds_;

    int targetId_;
    int classId_;
    cv::Rect bestBox_;
    int lostTargetCount_;
    static constexpr int MAX_LOST_FRAMES = 8;
    static constexpr float DISTANCE_THRESHOLD = 40.0f;
    static constexpr int FRAMES_OUTSIDE_LANE = 8;
    static constexpr float CONFIDENCE_THRESHOLD = 0.8f;
    int framesCurrentTargetOutsideLane_;
    int noSpeedLimitFrames_;

    VideoRecorder videoRecorder_;
    HUDRenderer hudRenderer_;
    DataLogger dataLogger_;

    std::deque<double> processing_times_;
    int total_detections_;

    bool emergency_stop_;
    double last_detection_time_;
    static constexpr double DETECTION_TIMEOUT = 2.0;

    EgoVehicle egoVehicle_;
    TrafficSignStabilizer speedLimitStabilizer_;

    void initializeEnhancedFeatures();
    void createDirectory(const std::string &path);
    std::string getModelPath();
    void publishEnhancedData(float ego_speed, const std::string &action_str,
                             float throttle_cmd, float brake_cmd);
    void checkSafetyConditions();
    void processEnhancedFrame(cv::Mat &image);
    void
    performEnhancedTargetSelection(const std::vector<STrack> &outputStracks,
                                   const std::vector<cv::Vec4i> &lanes,
                                   cv::Mat &image);
    void executeEnhancedTargetSwitching(
        int detectedTargetId, int detectedClassId, cv::Rect bestBoxTmp,
        bool currentTargetStillExists, bool currentTargetInLane,
        float maxBottomY, float currentTargetBottomY);
    void updateSpeedLimits(const std::vector<STrack> &outputStracks);
    void updateFPS();

public:
    EnhancedVideoSubscriber();
    ~EnhancedVideoSubscriber();
    void imageCallback(const sensor_msgs::Image::ConstPtr &msg);
};

#endif // ENHANCED_VIDEO_SUBSCRIBER_H