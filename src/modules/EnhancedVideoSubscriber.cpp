#include "modules/EnhancedVideoSubscriber.h"
#include <iomanip>
#include <ros/ros.h>
#include <sstream>
#include <sys/stat.h>
#include <unistd.h>
using namespace std;
using namespace Config;
EnhancedVideoSubscriber::EnhancedVideoSubscriber()
    : it_(nh_), model_(getModelPath(), Logger::getInstance()), laneDetector_(),
      tracker_(30.0, 30), frameCount_(0), fps_(30.0), maxSpeed_(-1),
      accSpeed_(config.speedControl.cruiseSpeedKph), noSpeedLimitFrames_(0),
      speedLimitStabilizer_(6),
      currentEgoSpeed_(config.speedControl.initialSpeedKph),
      lastSpeedUpdateTime_(0), targetId_(-1), classId_(-1), lostTargetCount_(0),
      framesCurrentTargetOutsideLane_(0) {
    fpsStartTime_ = chrono::steady_clock::now();
    initializeEnhancedFeatures();
    image_sub_ = it_.subscribe("video/image", 1,
                               &EnhancedVideoSubscriber::imageCallback, this);
    ROS_INFO("Enhanced video subscriber started");
    ROS_INFO("Model loaded: %s", model_path_.c_str());
}

EnhancedVideoSubscriber::~EnhancedVideoSubscriber() { cv::destroyAllWindows(); }

void EnhancedVideoSubscriber::initializeEnhancedFeatures() {
    speed_pub_ = nh_.advertise<std_msgs::Float32>("ego_speed", 10);
    action_pub_ = nh_.advertise<std_msgs::String>("driving_action", 10);
    throttle_pub_ = nh_.advertise<std_msgs::Float32>("control/throttle", 10);
    brake_pub_ = nh_.advertise<std_msgs::Float32>("control/brake", 10);

    ros::NodeHandle private_nh("~");
    private_nh.param<std::string>("output_dir", output_dir_,
                                  "/tmp/driving_logs");
    private_nh.param<bool>("enable_debug_output", enable_debug_output_, true);
    private_nh.param<bool>("enable_data_logging", enable_data_logging_, true);

    createDirectory(output_dir_);

    if (enable_data_logging_) {
        dataLogger_.init(output_dir_);
    }

    processing_times_.clear();
    total_detections_ = 0;
    emergency_stop_ = false;
    last_detection_time_ = ros::Time::now().toSec();

    ROS_INFO("Enhanced features initialized:");
    ROS_INFO("- Debug output: %s",
             enable_debug_output_ ? "enabled" : "disabled");
    ROS_INFO("- Data logging: %s",
             enable_data_logging_ ? "enabled" : "disabled");
    ROS_INFO("- Output directory: %s", output_dir_.c_str());
}

void EnhancedVideoSubscriber::createDirectory(const std::string &path) {
    struct stat info;
    if (stat(path.c_str(), &info) != 0) {
        if (mkdir(path.c_str(), 0755) != 0) {
            ROS_WARN("Could not create directory: %s", path.c_str());
        }
    }
}

std::string EnhancedVideoSubscriber::getModelPath() {
    ros::NodeHandle private_nh("~");
    private_nh.param<std::string>("model_path", model_path_, "");

    if (model_path_.empty()) {
        nh_.param<std::string>("model_path", model_path_, "");
    }

    if (model_path_.empty()) {
        ROS_ERROR(
            "No model path specified! Use: model_path:=/path/to/model.engine");
        ros::shutdown();
        return "";
    }

    return model_path_;
}

void EnhancedVideoSubscriber::publishEnhancedData(float ego_speed,
                                                  const std::string &action_str,
                                                  float throttle_cmd,
                                                  float brake_cmd) {
    std_msgs::Float32 speed_msg;
    speed_msg.data = ego_speed;
    speed_pub_.publish(speed_msg);

    std_msgs::String action_msg;
    action_msg.data = action_str;
    action_pub_.publish(action_msg);

    std_msgs::Float32 throttle_msg;
    throttle_msg.data = throttle_cmd;
    throttle_pub_.publish(throttle_msg);

    std_msgs::Float32 brake_msg;
    brake_msg.data = brake_cmd;
    brake_pub_.publish(brake_msg);
}

void EnhancedVideoSubscriber::checkSafetyConditions() {
    double current_time = ros::Time::now().toSec();

    if (current_time - last_detection_time_ > DETECTION_TIMEOUT) {
        if (!emergency_stop_) {
            emergency_stop_ = true;
            ROS_WARN("Emergency stop activated: No detections for %.1f seconds",
                     current_time - last_detection_time_);
        }
    } else {
        emergency_stop_ = false;
    }

    if (currentEgoSpeed_ > maxSpeed_ * 1.2 && maxSpeed_ > 0) {
        // ROS_WARN("Speed limit exceeded: %.1f km/h (limit: %d km/h)",
        //          currentEgoSpeed_, maxSpeed_);
    }
}

void EnhancedVideoSubscriber::imageCallback(
    const sensor_msgs::Image::ConstPtr &msg) {

    try {
        cv_bridge::CvImagePtr cv_ptr =
            cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        cv::Mat image = cv_ptr->image;

        if (image.empty()) {
            ROS_WARN("Received empty image");
            return;
        }

        if (!videoRecorder_.isInitialized()) {
            videoRecorder_.init(image, output_dir_);
        }
        // ROS_INFO("Output file in %s", output_dir_.c_str());

        processEnhancedFrame(image);
        videoRecorder_.writeFrame(image);

    } catch (cv_bridge::Exception &e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
    } catch (const std::exception &e) {
        ROS_ERROR("Processing exception: %s", e.what());
    }
}

void EnhancedVideoSubscriber::processEnhancedFrame(cv::Mat &image) {
    auto start = chrono::high_resolution_clock::now();
    double timeStart = getCurrentTimeInSeconds();

    checkSafetyConditions();

    std::vector<Detection> res;
    model_.preprocess(image);
    model_.infer();
    model_.postProcess(image, res);

    std::vector<Object> objects = filterDetections(res);
    std::vector<cv::Vec4i> lanes = laneDetector_.detectLanes(image);

    if (!objects.empty()) {
        last_detection_time_ = ros::Time::now().toSec();
    }

    std::vector<STrack> outputStracks = tracker_.update(objects);
    total_detections_ += outputStracks.size();

    performEnhancedTargetSelection(outputStracks, lanes, image);

    cv::Scalar actionColor = cv::Scalar(0, 255, 0);
    float avgDistance = 0.0f;
    float frontAbsoluteSpeed = 0.0f;
    bool accActive = isTrackingClass(classId_);
    egoVehicle_.updateSpeedControl(
        timeStart, targetId_, bestBox_, currentEgoSpeed_, lastSpeedUpdateTime_,
        objectBuffers_, prevDistances_, prevTimes_, smoothedSpeeds_,
        speedChangeHistory_, avgDistance, frontAbsoluteSpeed, actionColor);

    updateSpeedLimits(outputStracks);

    model_.draw(image, outputStracks);
    laneDetector_.drawLanes(image, lanes);

    updateFPS();

    hudRenderer_.setEmergencyStop(emergency_stop_);
    hudRenderer_.render(
        image, currentEgoSpeed_, accSpeed_, frontAbsoluteSpeed, avgDistance,
        accActive, egoVehicle_.getAction(), actionColor, fps_, targetId_,
        egoVehicle_.getEngineForce(), egoVehicle_.getThrottleForce(),
        egoVehicle_.getBrakeForce());

    publishEnhancedData(currentEgoSpeed_, egoVehicle_.getAction(),
                        egoVehicle_.getThrottleCmd(),
                        egoVehicle_.getBrakeCmd());

    if (enable_data_logging_) {
        auto end = chrono::high_resolution_clock::now();
        double processing_time = chrono::duration<double>(end - start).count();
        dataLogger_.log(timeStart, frameCount_, fps_, currentEgoSpeed_,
                        targetId_, egoVehicle_.getAction(),
                        outputStracks.size(), avgDistance, frontAbsoluteSpeed,
                        processing_time, maxSpeed_, accSpeed_);
    }

    if (enable_debug_output_) {
        cv::imshow("Enhanced Driving Assistant", image);
        cv::waitKey(1);
    }
}

void EnhancedVideoSubscriber::performEnhancedTargetSelection(
    const std::vector<STrack> &outputStracks,
    const std::vector<cv::Vec4i> &lanes, cv::Mat &image) {
    int detectedTargetId = -1;
    int detectedClassId = -1;
    cv::Rect bestBoxTmp;
    float maxBottomY = -1;
    float maxConfidence = 0;
    bool currentTargetStillExists = false;
    bool currentTargetInLane = false;
    float currentTargetBottomY = -1;

    if (targetId_ != -1) {
        auto it = std::find_if(
            outputStracks.begin(), outputStracks.end(),
            [this](const STrack &obj) { return obj.track_id == targetId_; });

        if (it != outputStracks.end()) {
            currentTargetStillExists = true;
            const auto &tlbr = it->tlbr;
            bestBox_ = cv::Rect(tlbr[0], tlbr[1], tlbr[2] - tlbr[0],
                                tlbr[3] - tlbr[1]);
            currentTargetBottomY = tlbr[3];
        }
    }

    for (const STrack &obj : outputStracks) {
        const auto &tlbr = obj.tlbr;
        float h = tlbr[3] - tlbr[1];

        if (h > 400 || obj.score < CONFIDENCE_THRESHOLD)
            continue;

        int classId = obj.classId;

        if ((classId == 2 || classId == 4 || classId == 5) &&
            lanes.size() >= 2) {
            cv::Point bottom_center((tlbr[0] + tlbr[2]) / 2.0f, tlbr[3]);
            std::vector<cv::Point> lane_area = {{lanes[0][0], lanes[0][1]},
                                                {lanes[1][0], lanes[1][1]},
                                                {lanes[1][2], lanes[1][3]},
                                                {lanes[0][2], lanes[0][3]}};

            if (cv::pointPolygonTest(lane_area, bottom_center, false) >= 0) {
                cv::Point center((tlbr[0] + tlbr[2]) / 2.0f,
                                 (tlbr[1] + tlbr[3]) / 2.0f);
                cv::circle(image, center, 5, cv::Scalar(0, 255, 0), -1);

                if (obj.track_id == targetId_) {
                    currentTargetInLane = true;
                    lostTargetCount_ = 0;
                    framesCurrentTargetOutsideLane_ = 0;
                } else if (tlbr[3] > maxBottomY && obj.score > maxConfidence) {
                    bestBoxTmp =
                        cv::Rect(tlbr[0], tlbr[1], tlbr[2] - tlbr[0], h);
                    detectedTargetId = obj.track_id;
                    detectedClassId = obj.classId;
                    maxBottomY = tlbr[3];
                    maxConfidence = obj.score;
                }
            }
        }
    }

    if (targetId_ != -1 && !currentTargetInLane) {
        framesCurrentTargetOutsideLane_++;
    }

    executeEnhancedTargetSwitching(
        detectedTargetId, detectedClassId, bestBoxTmp, currentTargetStillExists,
        currentTargetInLane, maxBottomY, currentTargetBottomY);
}

void EnhancedVideoSubscriber::executeEnhancedTargetSwitching(
    int detectedTargetId, int detectedClassId, cv::Rect bestBoxTmp,
    bool currentTargetStillExists, bool currentTargetInLane, float maxBottomY,
    float currentTargetBottomY) {
    bool shouldSwitchTarget = false;
    std::string switchReason = "";

    if (targetId_ == -1) {
        if (detectedTargetId != -1) {
            shouldSwitchTarget = true;
            switchReason = "No current target";
        }
    } else if (!currentTargetStillExists) {
        lostTargetCount_++;
        if (lostTargetCount_ >= MAX_LOST_FRAMES) {
            if (detectedTargetId != -1) {
                shouldSwitchTarget = true;
                switchReason = "Current target lost";
            } else {
                targetId_ = -1;
                classId_ = -1;
                lostTargetCount_ = 0;
            }
        }
    } else if (detectedTargetId != -1 && detectedTargetId != targetId_) {
        if (framesCurrentTargetOutsideLane_ >= FRAMES_OUTSIDE_LANE) {
            shouldSwitchTarget = true;
            switchReason = "Target outside lane too long";
        } else if (currentTargetInLane &&
                   (maxBottomY - currentTargetBottomY) > DISTANCE_THRESHOLD) {
            shouldSwitchTarget = true;
            switchReason = "Closer target available";
        } else if (!currentTargetInLane) {
            shouldSwitchTarget = true;
            switchReason = "Better target in lane";
        }
    }

    if (shouldSwitchTarget && detectedTargetId != -1) {
        targetId_ = detectedTargetId;
        classId_ = detectedClassId;
        bestBox_ = bestBoxTmp;
        lostTargetCount_ = 0;
        framesCurrentTargetOutsideLane_ = 0;

        if (enable_debug_output_) {
            // ROS_INFO("Target switched to ID %d - Reason: %s", targetId_,
            //          switchReason.c_str());
        }
    }
}

void EnhancedVideoSubscriber::updateSpeedLimits(
    const std::vector<STrack> &outputStracks) {
    bool detectedSpeedLimitThisFrame = false;
    for (const STrack &obj : outputStracks) {
        int classId = obj.classId;
        float conf = obj.score;
        float x1 = obj.tlbr[0];
        float y1 = obj.tlbr[1];
        float x2 = obj.tlbr[2];
        float y2 = obj.tlbr[3];

        int width = static_cast<int>(x2 - x1);
        int height = static_cast<int>(y2 - y1);
        int center_x = static_cast<int>((x1 + x2) / 2.0f);
        int center_y = static_cast<int>((y1 + y2) / 2.0f);

        if (classId >= 12 && classId <= 17 && conf > CONFIDENCE_THRESHOLD &&
            width >= 22) {

            detectedSpeedLimitThisFrame = true;
            int detectedSpeed = (classId - 9) * 10;
            std::string stableSpeedStr =
                speedLimitStabilizer_.update(std::to_string(detectedSpeed));
            int stableSpeed = std::stoi(stableSpeedStr);

            if (stableSpeed != maxSpeed_) {
                maxSpeed_ = stableSpeed;
                if (enable_debug_output_) {
                    ROS_INFO("📸 Stabilized speed limit updated: %d km/h",
                             maxSpeed_);
                }
            }
        }
    }

    if (detectedSpeedLimitThisFrame) {
        noSpeedLimitFrames_ = 0;
    } else {
        noSpeedLimitFrames_++;
    }

    if (noSpeedLimitFrames_ >= 20) {
        speedLimitStabilizer_.clear();
    }
    // if (maxSpeed_ != -1) {
    //     int targetSpeed =
    //         std::min(maxSpeed_, config.speedControl.cruiseSpeedKph);
    //     if (accSpeed_ < targetSpeed) {
    //         accSpeed_ = std::min(accSpeed_ + 1, targetSpeed);
    //     } else if (accSpeed_ > targetSpeed) {
    //         accSpeed_ = std::max(accSpeed_ - 1, 0);
    //     }
    // } else {
    //     accSpeed_ = config.speedControl.cruiseSpeedKph;
    // }
}

void EnhancedVideoSubscriber::updateFPS() {
    frameCount_++;
    auto now = chrono::steady_clock::now();
    auto elapsed =
        chrono::duration_cast<chrono::seconds>(now - fpsStartTime_).count();

    if (elapsed >= 1) {
        fps_ = frameCount_ / static_cast<double>(elapsed);
        frameCount_ = 0;
        fpsStartTime_ = now;
    }
}