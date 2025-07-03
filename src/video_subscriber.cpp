#include <app.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sys/stat.h>
#include <unistd.h>

#include <opencv2/opencv.hpp>

#include "EgoVehicle.h"

using namespace std;
using namespace Config;
class VideoSubscriber {
   private:
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    std::string model_path_;

    // Detection models
    Detect model_;
    LaneDetector laneDetector_;

    // Tracking variables
    BYTETracker tracker_;
    int frameCount_;
    std::chrono::steady_clock::time_point fpsStartTime_;
    double fps_;

    // Speed and control variables
    int maxSpeed_;  // km/h
    int accSpeed_;  // km/h
    float currentEgoSpeed_;
    double lastSpeedUpdateTime_;
    std::deque<float> speedChangeHistory_;
    std::deque<float> distanceHistory_;

    // Object tracking buffers
    std::map<int, std::deque<float>> objectBuffers_;
    std::map<int, float> prevDistances_;
    std::map<int, double> prevTimes_;
    std::map<int, float> smoothedSpeeds_;

    // Speed control logic
    std::string action = "FREE DRIVE";
    // Target tracking variables
    int targetId_;
    cv::Rect bestBox_;
    int lostTargetCount_;
    static constexpr int MAX_LOST_FRAMES = 5;

    // Switching criteria thresholds
    static constexpr float DISTANCE_THRESHOLD = 50.0f;  // pixels
    static constexpr int FRAMES_OUTSIDE_LANE = 10;      // frames
    int framesCurrentTargetOutsideLane_;

    std::string getModelPath() {
        ros::NodeHandle private_nh("~");
        private_nh.param<std::string>("model_path", model_path_, "");
        // If not found in private, try global
        if (model_path_.empty()) {
            nh_.param<std::string>("model_path", model_path_, "");
        }
        if (model_path_.empty()) {
            ROS_ERROR("No model path specified! Use: model_path:=/path/to/model.engine");
            ros::shutdown();
            return "";
        }
        return model_path_;
    }

   public:
    VideoSubscriber()
        : it_(nh_),
          model_(getModelPath(), Logger::getInstance()),
          laneDetector_(),
          tracker_(30.0, 30),  // fps=30, frame_rate=30
          frameCount_(0),
          fps_(30.0),
          maxSpeed_(-1),
          accSpeed_(60),
          currentEgoSpeed_(config.speedControl.initialSpeedKph),
          lastSpeedUpdateTime_(0),
          targetId_(-1),
          lostTargetCount_(0),
          framesCurrentTargetOutsideLane_(0) {
        // Initialize timing
        fpsStartTime_ = std::chrono::steady_clock::now();

        // Subscribe to video topic
        image_sub_ = it_.subscribe("video/image", 1, &VideoSubscriber::imageCallback, this);

        ROS_INFO("Video subscriber started. Waiting for frames...");
        ROS_INFO("Model loaded: %s", model_path_.c_str());
    }
    void imageCallback(const sensor_msgs::Image::ConstPtr &msg) {
        try {  // Convert ROS image to OpenCV format
            cv_bridge::CvImagePtr cv_ptr =
                cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
            cv::Mat image = cv_ptr->image;
            if (image.empty()) {
                ROS_WARN("Received empty image");
                return;
            }

            // Process the frame
            processFrame(image);
        } catch (cv_bridge::Exception &e) {
            ROS_ERROR("cv_bridge exception: %s", e.what());
        } catch (const std::exception &e) {
            ROS_ERROR("Processing exception: %s", e.what());
        }
    }

   private:
    void processFrame(cv::Mat &image) {
        auto now = std::chrono::steady_clock::now();
        auto start = std::chrono::system_clock::now();
        double timeStart = getCurrentTimeInSeconds();

        // Object detection
        std::vector<Detection> res;
        model_.preprocess(image);
        model_.infer();
        model_.postprocess(image, res);

        std::vector<Object> objects = filterDetections(res);
        std::vector<cv::Vec4i> lanes = laneDetector_.detectLanes(image);

        // Tracking
        std::vector<STrack> outputStracks = tracker_.update(objects);

        auto end = std::chrono::system_clock::now();

        model_.draw(image, outputStracks);

        // --- Ego Vehicle Speed Control Logic ---
        int detectedTargetId = -1;
        cv::Rect bestBoxTmp;
        float maxBottomY = -1;
        bool currentTargetStillExists = false;
        bool currentTargetInLane = false;
        float currentTargetBottomY = -1;

        // First, check if our current target still exists and update its bounding
        // box
        if (targetId_ != -1) {
            auto it = std::find_if(outputStracks.begin(), outputStracks.end(),
                                   [this](const STrack &obj) { return obj.track_id == targetId_; });

            if (it != outputStracks.end()) {
                currentTargetStillExists = true;
                const auto &tlbr = it->tlbr;
                bestBox_ = cv::Rect(tlbr[0], tlbr[1], tlbr[2] - tlbr[0], tlbr[3] - tlbr[1]);
                currentTargetBottomY = tlbr[3];
            }
        }

        // Log detected objects and find lane candidates
        for (const STrack &obj : outputStracks) {
            const auto &tlbr = obj.tlbr;
            float h = tlbr[3] - tlbr[1];
            if (h > 400) continue;

            int classId = obj.classId;
            float conf = obj.score;

            // Detect speed limit signs
            if (classId >= 12 && classId <= 17 && conf > 0.6f) {
                maxSpeed_ = (classId - 9) * 10;
            }

            // Detect vehicles in lane
            if ((classId == 2 || classId == 4 || classId == 5) && lanes.size() >= 2) {
                cv::Point bottom_center((tlbr[0] + tlbr[2]) / 2.0f, tlbr[3]);

                std::vector<cv::Point> lane_area = {{lanes[0][0], lanes[0][1]},
                                                    {lanes[1][0], lanes[1][1]},
                                                    {lanes[1][2], lanes[1][3]},
                                                    {lanes[0][2], lanes[0][3]}};

                // Check if vehicle is in lane
                if (cv::pointPolygonTest(lane_area, bottom_center, false) >= 0) {
                    cv::Point center((tlbr[0] + tlbr[2]) / 2.0f, (tlbr[1] + tlbr[3]) / 2.0f);
                    cv::circle(image, center, 5, cv::Scalar(0, 255, 0), -1);

                    // Check if this is our current target
                    if (obj.track_id == targetId_) {
                        currentTargetInLane = true;
                        lostTargetCount_ = 0;                 // Reset lost counter
                        framesCurrentTargetOutsideLane_ = 0;  // Reset outside lane counter
                    }
                    // Consider new targets (even if we have a current target)
                    else if (tlbr[3] > maxBottomY) {
                        bestBoxTmp = cv::Rect(tlbr[0], tlbr[1], tlbr[2] - tlbr[0], h);
                        detectedTargetId = obj.track_id;
                        maxBottomY = tlbr[3];  // update closest
                    }
                }
            }
        }

        // Update outside lane counter
        if (targetId_ != -1 && !currentTargetInLane) {
            framesCurrentTargetOutsideLane_++;
        }

        // Target switching logic
        bool shouldSwitchTarget = false;
        std::string switchReason = "";

        if (targetId_ == -1) {
            // No current target - assign new one if found in lane
            if (detectedTargetId != -1) {
                shouldSwitchTarget = true;
                switchReason = "No current target";
            }
        } else if (!currentTargetStillExists) {
            // Current target disappeared from tracking
            lostTargetCount_++;
            if (lostTargetCount_ >= MAX_LOST_FRAMES) {
                if (detectedTargetId != -1) {
                    shouldSwitchTarget = true;
                    switchReason = "Current target lost";
                } else {
                    targetId_ = -1;  // No replacement available
                    lostTargetCount_ = 0;
                }
            }
        } else if (detectedTargetId != -1 && detectedTargetId != targetId_) {
            // We have both current and new target candidates
            // Check switching criteria:

            // 1. Current target has been outside lane detection for too long
            if (framesCurrentTargetOutsideLane_ >= FRAMES_OUTSIDE_LANE) {
                shouldSwitchTarget = true;
                switchReason = "Current target outside lane too long";
            }
            // 2. New target is significantly closer (more relevant for following)
            else if (currentTargetInLane &&
                     (maxBottomY - currentTargetBottomY) > DISTANCE_THRESHOLD) {
                shouldSwitchTarget = true;
                switchReason = "New target significantly closer";
            }
            // 3. Current target not in lane but new target is
            else if (!currentTargetInLane) {
                shouldSwitchTarget = true;
                switchReason = "New target in lane, current not";
            }
        }

        // Execute target switch if needed
        if (shouldSwitchTarget && detectedTargetId != -1) {
            targetId_ = detectedTargetId;
            bestBox_ = bestBoxTmp;
            lostTargetCount_ = 0;
            framesCurrentTargetOutsideLane_ = 0;

            // Debug output
            ROS_INFO("Target switched to ID %d - Reason: %s", targetId_, switchReason.c_str());
        }

        cv::Scalar actionColor = cv::Scalar(0, 255, 0);
        float avgDistance = 0.0f;
        float frontAbsoluteSpeed = 0.0f;
        EgoVehicle::updateSpeedControl(timeStart, targetId_, bestBox_, currentEgoSpeed_,
                                       lastSpeedUpdateTime_, objectBuffers_, prevDistances_,
                                       prevTimes_, smoothedSpeeds_, speedChangeHistory_,
                                       avgDistance, frontAbsoluteSpeed, action, actionColor);

        laneDetector_.drawLanes(image, lanes);

        // Speed limit control
        if (maxSpeed_ != -1) {
            if (accSpeed_ < maxSpeed_ && accSpeed_ < config.speedControl.cruiseSpeedKph) {
                accSpeed_ += 1;  // Increase speed by 1 km/h
            } else if (accSpeed_ > maxSpeed_ && accSpeed_ > 0) {
                accSpeed_ -= 1;  // Decrease speed by 1 km/h
            }
        } else {
            accSpeed_ = config.speedControl
                            .cruiseSpeedKph;  // Reset to max speed if no speed limit detected
        }

        // FPS calculation
        frameCount_++;
        auto elapsed =
            std::chrono::duration_cast<std::chrono::seconds>(now - fpsStartTime_).count();
        if (elapsed >= 1) {
            fps_ = frameCount_ / static_cast<double>(elapsed);
            frameCount_ = 0;
            fpsStartTime_ = now;
        }
        // Always display information on frame
        drawHUD(image, currentEgoSpeed_, accSpeed_, maxSpeed_, frontAbsoluteSpeed, avgDistance,
                action, actionColor, fps_, targetId_);
        // Display result (optional - you might want to publish instead)
        cv::imshow("Result", image);
        cv::waitKey(1);
    }
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "video_subscriber_test");

    try {
        // Initialize video subscriber object
        VideoSubscriber subscriber;

        ROS_INFO("Advanced video subscriber test running...");
        ROS_INFO("Subscribing to: /video/image");
        ROS_INFO("Features: Object detection, Lane detection, Speed control");

        // Load configuration from JSON file
        std::cout << "🔧 Loading configuration..." << std::endl;
        try {
            Config::loadConfig(
                "/home/trung/catkin_ws/src/thesis/config.json");  // This might throw if the file is
                                                                  // not found or invalid.
        } catch (const std::exception &e) {
            ROS_ERROR("Failed to load config.json: %s", e.what());
            return -1;
        }

        // Retrieve camera settings and log them
        CameraSettings cameraSettings = Config::config.camera;
        if (cameraSettings.focalLength == 0 || cameraSettings.realObjectWidth == 0) {
            ROS_WARN("Camera settings might be incomplete. Focal Length: %f, Real Object Width: %f",
                     cameraSettings.focalLength, cameraSettings.realObjectWidth);
        }

        std::cout << "Camera settings loaded:\n";
        std::cout << "Focal Length: " << cameraSettings.focalLength << " mm\n";
        std::cout << "Real Object Width: " << cameraSettings.realObjectWidth << " m\n";
        std::cout << "FPS: " << cameraSettings.fps << " FPS\n";

        std::cout << "✅ Configuration loaded successfully.\n";

        // Initialize the tracking estimator
        try {
            STrack::initializeEstimator();
        } catch (const std::exception &e) {
            ROS_ERROR("Failed to initialize tracking estimator: %s", e.what());
            return -1;
        }

        // Start processing
        ros::spin();
    } catch (const std::exception &e) {
        ROS_ERROR("Exception in main: %s", e.what());
        return -1;
    }

    // Clean up OpenCV windows if any
    cv::destroyAllWindows();
    return 0;
}