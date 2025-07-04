#include <app.h>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/Twist.h>
#include <image_transport/image_transport.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>
#include <sys/stat.h>
#include <unistd.h>

#include <TrafficSignStabilizer.hpp>
#include <chrono>
#include <fstream>
#include <iomanip>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <sstream>

#include "EgoVehicle.h"

using namespace std;
using namespace Config;

class EnhancedVideoSubscriber {
   private:
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;

    // Publishers for enhanced functionality
    ros::Publisher speed_pub_;
    ros::Publisher action_pub_;
    ros::Publisher cmd_vel_pub_;

    std::string model_path_;
    std::string output_dir_;
    bool enable_debug_output_;
    bool enable_data_logging_;

    // Detection models
    Detect model_;
    LaneDetector laneDetector_;

    // Tracking variables
    BYTETracker tracker_;
    int frameCount_;
    chrono::steady_clock::time_point fpsStartTime_;
    double fps_;

    // Enhanced speed and control variables
    int maxSpeed_;
    int accSpeed_;
    float currentEgoSpeed_;
    double lastSpeedUpdateTime_;
    std::deque<float> speedChangeHistory_;
    std::deque<float> distanceHistory_;

    // Object tracking buffers
    std::map<int, std::deque<float>> objectBuffers_;
    std::map<int, float> prevDistances_;
    std::map<int, double> prevTimes_;
    std::map<int, float> smoothedSpeeds_;

    // Enhanced control logic
    std::string action = "INITIALIZING";
    int targetId_;
    cv::Rect bestBox_;
    int lostTargetCount_;
    static constexpr int MAX_LOST_FRAMES = 8;  // Increased tolerance

    // Enhanced switching criteria
    static constexpr float DISTANCE_THRESHOLD = 40.0f;   // More sensitive
    static constexpr int FRAMES_OUTSIDE_LANE = 8;        // Reduced frames
    static constexpr float CONFIDENCE_THRESHOLD = 0.7f;  // Higher confidence
    int framesCurrentTargetOutsideLane_;
    int noSpeedLimitFrames_;
    // Video recording
    bool is_writer_initialized_ = false;
    VideoWriter writer_;
    std::string filename_;

    // Data logging
    std::ofstream log_file_;
    std::string log_filename_;

    // Performance metrics
    std::deque<double> processing_times_;
    double avg_processing_time_;
    int total_detections_;

    // Safety features
    bool emergency_stop_;
    double last_detection_time_;
    static constexpr double DETECTION_TIMEOUT = 2.0;  // seconds

    TrafficSignStabilizer speedLimitStabilizer_;
    void initializeEnhancedFeatures() {
        // Initialize publishers
        speed_pub_ = nh_.advertise<std_msgs::Float32>("ego_speed", 10);
        action_pub_ = nh_.advertise<std_msgs::String>("driving_action", 10);

        // Load enhanced parameters
        ros::NodeHandle private_nh("~");
        private_nh.param<std::string>("output_dir", output_dir_, "/tmp/driving_logs");
        private_nh.param<bool>("enable_debug_output", enable_debug_output_, true);
        private_nh.param<bool>("enable_data_logging", enable_data_logging_, true);

        // Create output directory
        createDirectory(output_dir_);

        // Initialize data logging
        if (enable_data_logging_) {
            initializeDataLogging();
        }

        // Initialize performance tracking
        processing_times_.clear();
        total_detections_ = 0;
        emergency_stop_ = false;
        last_detection_time_ = ros::Time::now().toSec();

        ROS_INFO("Enhanced features initialized:");
        ROS_INFO("- Debug output: %s", enable_debug_output_ ? "enabled" : "disabled");
        ROS_INFO("- Data logging: %s", enable_data_logging_ ? "enabled" : "disabled");
        ROS_INFO("- Output directory: %s", output_dir_.c_str());
    }

    void createDirectory(const std::string &path) {
        struct stat info;
        if (stat(path.c_str(), &info) != 0) {
            if (mkdir(path.c_str(), 0755) != 0) {
                ROS_WARN("Could not create directory: %s", path.c_str());
            }
        }
    }

    void initializeDataLogging() {
        auto now = chrono::system_clock::now();
        auto time_t = chrono::system_clock::to_time_t(now);
        std::stringstream ss;
        ss << output_dir_ << "/driving_log_"
           << std::put_time(std::localtime(&time_t), "%Y%m%d_%H%M%S") << ".csv";
        log_filename_ = ss.str();

        log_file_.open(log_filename_);
        if (log_file_.is_open()) {
            // Write CSV header
            log_file_ << "timestamp,frame_count,fps,ego_speed,target_id,action,detection_count,"
                      << "avg_distance,front_speed,processing_time,max_speed,acc_speed\n";
            ROS_INFO("Data logging initialized: %s", log_filename_.c_str());
        } else {
            ROS_ERROR("Failed to open log file: %s", log_filename_.c_str());
        }
    }

    void initializeVideoWriter(const cv::Mat &first_frame) {
        if (is_writer_initialized_) return;

        ros::NodeHandle private_nh("~");
        private_nh.param<std::string>("output_file", filename_, "");

        if (filename_.empty()) {
            auto now = chrono::system_clock::now();
            auto time_t = chrono::system_clock::to_time_t(now);
            std::stringstream ss;
            ss << output_dir_ << "/recording_"
               << std::put_time(std::localtime(&time_t), "%Y%m%d_%H%M%S") << ".avi";
            filename_ = ss.str();
        }

        bool isColor = (first_frame.type() == CV_8UC3);
        int codec = cv::VideoWriter::fourcc('H', '2', '6', '4');  // Better codec
        double fps = 30.0;

        writer_.open(filename_, codec, fps, first_frame.size(), isColor);
        if (!writer_.isOpened()) {
            ROS_ERROR("Could not open video file: %s", filename_.c_str());
            return;
        }

        is_writer_initialized_ = true;
        ROS_INFO("Video recording started: %s", filename_.c_str());
    }

    std::string getModelPath() {
        ros::NodeHandle private_nh("~");
        private_nh.param<std::string>("model_path", model_path_, "");

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

    void publishEnhancedData(float ego_speed, const std::string &action_str) {
        // Publish ego speed
        std_msgs::Float32 speed_msg;
        speed_msg.data = ego_speed;
        speed_pub_.publish(speed_msg);

        // Publish driving action
        std_msgs::String action_msg;
        action_msg.data = action_str;
        action_pub_.publish(action_msg);
    }

    void logData(double timestamp, int detection_count, float avg_distance, float front_speed,
                 double processing_time) {
        if (!log_file_.is_open()) return;

        log_file_ << std::fixed << std::setprecision(3) << timestamp << "," << frameCount_ << ","
                  << fps_ << "," << currentEgoSpeed_ << "," << targetId_ << "," << action << ","
                  << detection_count << "," << avg_distance << "," << front_speed << ","
                  << processing_time << "," << maxSpeed_ << "," << accSpeed_ << "\n";

        log_file_.flush();  // Ensure data is written immediately
    }

    void updatePerformanceMetrics(double processing_time) {
        processing_times_.push_back(processing_time);
        if (processing_times_.size() > 100) {
            processing_times_.pop_front();
        }

        // Calculate average processing time
        double sum = 0.0;
        for (double time : processing_times_) {
            sum += time;
        }
        avg_processing_time_ = sum / processing_times_.size();
    }

    void drawEnhancedHUD(cv::Mat &image, float ego_speed, int acc_speed, int max_speed,
                         float front_speed, float avg_distance, const std::string &action_str,
                         cv::Scalar action_color, double fps, int target_id, int detection_count) {
        // Enhanced HUD with better styling
        int font = cv::FONT_HERSHEY_SIMPLEX;
        double font_scale = 0.6;
        int thickness = 2;

        // Background for HUD
        cv::rectangle(image, cv::Point(10, 10), cv::Point(420, 250), cv::Scalar(0, 0, 0, 128), -1);

        // Title
        cv::putText(image, "Enhanced Driving Assistant", cv::Point(20, 35), font, 0.7,
                    cv::Scalar(255, 255, 255), 2);

        // Speed information
        cv::putText(image, "Speed Control:", cv::Point(20, 65), font, font_scale,
                    cv::Scalar(255, 255, 255), thickness);
        cv::putText(image, "Ego: " + std::to_string((int)ego_speed) + " km/h", cv::Point(20, 85),
                    font, font_scale, cv::Scalar(0, 255, 0), thickness);
        cv::putText(image, "Target: " + std::to_string(acc_speed) + " km/h", cv::Point(20, 105),
                    font, font_scale, cv::Scalar(255, 255, 0), thickness);

        if (max_speed != -1) {
            cv::putText(image, "Limit: " + std::to_string(max_speed) + " km/h", cv::Point(20, 125),
                        font, font_scale, cv::Scalar(255, 0, 0), thickness);
        }

        // Action and target information
        cv::putText(image, "Action: " + action_str, cv::Point(20, 155), font, font_scale,
                    action_color, thickness);
        cv::putText(image, "Target ID: " + std::to_string(target_id), cv::Point(20, 175), font,
                    font_scale, cv::Scalar(255, 255, 255), thickness);

        // Performance metrics
        cv::putText(image, "FPS: " + std::to_string((int)fps), cv::Point(20, 195), font, font_scale,
                    cv::Scalar(255, 255, 255), thickness);
        cv::putText(image, "Detections: " + std::to_string(detection_count), cv::Point(20, 215),
                    font, font_scale, cv::Scalar(255, 255, 255), thickness);
        cv::putText(image,
                    "Proc Time: " + std::to_string((int)(avg_processing_time_ * 1000)) + "ms",
                    cv::Point(20, 235), font, font_scale, cv::Scalar(255, 255, 255), thickness);

        // Safety indicators
        if (emergency_stop_) {
            cv::rectangle(image, cv::Point(image.cols - 150, 10), cv::Point(image.cols - 10, 50),
                          cv::Scalar(0, 0, 255), -1);
            cv::putText(image, "EMERGENCY STOP", cv::Point(image.cols - 145, 35), font, 0.5,
                        cv::Scalar(255, 255, 255), 2);
        }

        // Distance information if available
        if (avg_distance > 0) {
            cv::putText(image, "Distance: " + std::to_string((int)avg_distance) + "m",
                        cv::Point(200, 85), font, font_scale, cv::Scalar(255, 255, 255), thickness);
        }

        if (front_speed > 0) {
            cv::putText(image, "Front Speed: " + std::to_string((int)front_speed) + " km/h",
                        cv::Point(200, 105), font, font_scale, cv::Scalar(255, 255, 255),
                        thickness);
        }
    }

    void checkSafetyConditions() {
        double current_time = ros::Time::now().toSec();

        // Check for detection timeout
        if (current_time - last_detection_time_ > DETECTION_TIMEOUT) {
            if (!emergency_stop_) {
                emergency_stop_ = true;
                ROS_WARN("Emergency stop activated: No detections for %.1f seconds",
                         current_time - last_detection_time_);
            }
        } else {
            emergency_stop_ = false;
        }

        // Check for other safety conditions
        if (currentEgoSpeed_ > maxSpeed_ * 1.2 && maxSpeed_ > 0) {
            // ROS_WARN("Speed limit exceeded: %.1f km/h (limit: %d km/h)", currentEgoSpeed_,
            //          maxSpeed_);
        }
    }

   public:
    EnhancedVideoSubscriber()
        : it_(nh_),
          model_(getModelPath(), Logger::getInstance()),
          laneDetector_(),
          tracker_(30.0, 30),
          frameCount_(0),
          fps_(30.0),
          maxSpeed_(-1),
          accSpeed_(60),
          noSpeedLimitFrames_(0),
          speedLimitStabilizer_(6),
          currentEgoSpeed_(config.speedControl.initialSpeedKph),
          lastSpeedUpdateTime_(0),
          targetId_(-1),
          lostTargetCount_(0),
          framesCurrentTargetOutsideLane_(0) {
        // Initialize timing
        fpsStartTime_ = chrono::steady_clock::now();

        // Initialize enhanced features
        initializeEnhancedFeatures();

        // Subscribe to video topic
        image_sub_ = it_.subscribe("video/image", 1, &EnhancedVideoSubscriber::imageCallback, this);

        ROS_INFO("Enhanced video subscriber started");
        ROS_INFO("Model loaded: %s", model_path_.c_str());
    }

    ~EnhancedVideoSubscriber() {
        if (log_file_.is_open()) {
            log_file_.close();
        }
        if (writer_.isOpened()) {
            writer_.release();
        }
        cv::destroyAllWindows();
    }

    void imageCallback(const sensor_msgs::Image::ConstPtr &msg) {
        auto callback_start = chrono::high_resolution_clock::now();

        try {
            cv_bridge::CvImagePtr cv_ptr =
                cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
            cv::Mat image = cv_ptr->image;

            if (image.empty()) {
                ROS_WARN("Received empty image");
                return;
            }

            if (!is_writer_initialized_) {
                initializeVideoWriter(image);
            }
            std::cout << "ThrottleCmd: " << EgoVehicle::getThrottleCmd()
                      << " BrakeCmd: " << EgoVehicle::getBrakeCmd() << std::endl;
            // Process the frame
            processEnhancedFrame(image);

            // Write frame if writer is ready
            if (writer_.isOpened()) {
                writer_ << image;
            }

        } catch (cv_bridge::Exception &e) {
            ROS_ERROR("cv_bridge exception: %s", e.what());
        } catch (const std::exception &e) {
            ROS_ERROR("Processing exception: %s", e.what());
        }

        auto callback_end = chrono::high_resolution_clock::now();
        double callback_time = chrono::duration<double>(callback_end - callback_start).count();
        updatePerformanceMetrics(callback_time);
    }

   private:
    void processEnhancedFrame(cv::Mat &image) {
        auto start = chrono::high_resolution_clock::now();
        double timeStart = getCurrentTimeInSeconds();

        // Safety check
        checkSafetyConditions();

        // Object detection
        std::vector<Detection> res;
        model_.preprocess(image);
        model_.infer();
        model_.postprocess(image, res);

        std::vector<Object> objects = filterDetections(res);
        std::vector<cv::Vec4i> lanes = laneDetector_.detectLanes(image);

        // Update last detection time
        if (!objects.empty()) {
            last_detection_time_ = ros::Time::now().toSec();
        }

        // Tracking
        std::vector<STrack> outputStracks = tracker_.update(objects);
        total_detections_ += outputStracks.size();

        // Enhanced target selection and tracking
        performEnhancedTargetSelection(outputStracks, lanes, image);

        // Enhanced speed control
        cv::Scalar actionColor = cv::Scalar(0, 255, 0);
        float avgDistance = 0.0f;
        float frontAbsoluteSpeed = 0.0f;

        EgoVehicle::updateSpeedControl(timeStart, targetId_, bestBox_, currentEgoSpeed_,
                                       lastSpeedUpdateTime_, objectBuffers_, prevDistances_,
                                       prevTimes_, smoothedSpeeds_, speedChangeHistory_,
                                       avgDistance, frontAbsoluteSpeed, action, actionColor);

        // Enhanced speed limit control
        updateSpeedLimits(outputStracks);

        // Draw enhanced visualization
        model_.draw(image, outputStracks);
        laneDetector_.drawLanes(image, lanes);

        // FPS calculation
        updateFPS();

        // Draw enhanced HUD
        drawEnhancedHUD(image, currentEgoSpeed_, accSpeed_, maxSpeed_, frontAbsoluteSpeed,
                        avgDistance, action, actionColor, fps_, targetId_, outputStracks.size());

        // Publish enhanced data
        publishEnhancedData(currentEgoSpeed_, action);

        // Log data
        if (enable_data_logging_) {
            auto end = chrono::high_resolution_clock::now();
            double processing_time = chrono::duration<double>(end - start).count();
            logData(timeStart, outputStracks.size(), avgDistance, frontAbsoluteSpeed,
                    processing_time);
        }

        // Display result
        if (enable_debug_output_) {
            cv::imshow("Enhanced Driving Assistant", image);
            cv::waitKey(1);
        }
    }

    void performEnhancedTargetSelection(const std::vector<STrack> &outputStracks,
                                        const std::vector<cv::Vec4i> &lanes, cv::Mat &image) {
        // Enhanced target selection logic with improved criteria
        int detectedTargetId = -1;
        cv::Rect bestBoxTmp;
        float maxBottomY = -1;
        float maxConfidence = 0;
        bool currentTargetStillExists = false;
        bool currentTargetInLane = false;
        float currentTargetBottomY = -1;

        // Check current target status
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

        // Enhanced target detection
        for (const STrack &obj : outputStracks) {
            const auto &tlbr = obj.tlbr;
            float h = tlbr[3] - tlbr[1];

            if (h > 400 || obj.score < CONFIDENCE_THRESHOLD) continue;

            int classId = obj.classId;

            // Vehicle classes with enhanced filtering
            if ((classId == 2 || classId == 4 || classId == 5) && lanes.size() >= 2) {
                cv::Point bottom_center((tlbr[0] + tlbr[2]) / 2.0f, tlbr[3]);

                std::vector<cv::Point> lane_area = {{lanes[0][0], lanes[0][1]},
                                                    {lanes[1][0], lanes[1][1]},
                                                    {lanes[1][2], lanes[1][3]},
                                                    {lanes[0][2], lanes[0][3]}};

                if (cv::pointPolygonTest(lane_area, bottom_center, false) >= 0) {
                    cv::Point center((tlbr[0] + tlbr[2]) / 2.0f, (tlbr[1] + tlbr[3]) / 2.0f);
                    cv::circle(image, center, 5, cv::Scalar(0, 255, 0), -1);

                    if (obj.track_id == targetId_) {
                        currentTargetInLane = true;
                        lostTargetCount_ = 0;
                        framesCurrentTargetOutsideLane_ = 0;
                    }
                    // Enhanced target selection criteria
                    else if (tlbr[3] > maxBottomY && obj.score > maxConfidence) {
                        bestBoxTmp = cv::Rect(tlbr[0], tlbr[1], tlbr[2] - tlbr[0], h);
                        detectedTargetId = obj.track_id;
                        maxBottomY = tlbr[3];
                        maxConfidence = obj.score;
                    }
                }
            }
        }

        // Update counters
        if (targetId_ != -1 && !currentTargetInLane) {
            framesCurrentTargetOutsideLane_++;
        }

        // Enhanced target switching logic
        executeEnhancedTargetSwitching(detectedTargetId, bestBoxTmp, currentTargetStillExists,
                                       currentTargetInLane, maxBottomY, currentTargetBottomY);
    }

    void executeEnhancedTargetSwitching(int detectedTargetId, cv::Rect bestBoxTmp,
                                        bool currentTargetStillExists, bool currentTargetInLane,
                                        float maxBottomY, float currentTargetBottomY) {
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
                    lostTargetCount_ = 0;
                    action = "FREE DRIVE";
                }
            }
        } else if (detectedTargetId != -1 && detectedTargetId != targetId_) {
            // Enhanced switching criteria
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
            bestBox_ = bestBoxTmp;
            lostTargetCount_ = 0;
            framesCurrentTargetOutsideLane_ = 0;

            if (enable_debug_output_) {
                ROS_INFO("Target switched to ID %d - Reason: %s", targetId_, switchReason.c_str());
            }
        }
    }

    void updateSpeedLimits(const std::vector<STrack> &outputStracks) {
        bool detectedSpeedLimitThisFrame = false;
        for (const STrack &obj : outputStracks) {
            int classId = obj.classId;
            float conf = obj.score;

            // Detect only speed limit signs
            if (classId >= 12 && classId <= 17 && conf > CONFIDENCE_THRESHOLD) {
                detectedSpeedLimitThisFrame = true;
                int detectedSpeed = (classId - 9) * 10;
                // Stabilize speed using voting
                std::string stableSpeedStr =
                    speedLimitStabilizer_.update(std::to_string(detectedSpeed));
                int stableSpeed = std::stoi(stableSpeedStr);

                if (stableSpeed != maxSpeed_) {
                    maxSpeed_ = stableSpeed;
                    if (enable_debug_output_) {
                        ROS_INFO("📸 Stabilized speed limit updated: %d km/h", maxSpeed_);
                    }
                }
            }
        }
        // Update detection history
        if (detectedSpeedLimitThisFrame) {
            noSpeedLimitFrames_ = 0;
        } else {
            noSpeedLimitFrames_++;
        }

        // Reset max speed if no sign detected for 20 frames
        if (noSpeedLimitFrames_ >= 20) {
            speedLimitStabilizer_.clear();  // Optionally clear stabilizer votes
        }
        // Speed control logic
        if (maxSpeed_ != -1) {
            int targetSpeed = std::min(maxSpeed_, config.speedControl.cruiseSpeedKph);
            if (accSpeed_ < targetSpeed) {
                accSpeed_ = std::min(accSpeed_ + 1, targetSpeed);
            } else if (accSpeed_ > targetSpeed) {
                accSpeed_ = std::max(accSpeed_ - 1, 0);
            }
        } else {
            accSpeed_ = config.speedControl.cruiseSpeedKph;
        }
    }

    void updateFPS() {
        frameCount_++;
        auto now = chrono::steady_clock::now();
        auto elapsed = chrono::duration_cast<chrono::seconds>(now - fpsStartTime_).count();

        if (elapsed >= 1) {
            fps_ = frameCount_ / static_cast<double>(elapsed);
            frameCount_ = 0;
            fpsStartTime_ = now;
        }
    }
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "enhanced_video_subscriber");

    try {
        ROS_INFO("Starting Enhanced Video Subscriber...");
        ROS_INFO("Features: Enhanced detection, lane tracking, autonomous control, data logging");

        // Load configuration
        std::cout << "🔧 Loading configuration..." << std::endl;
        try {
            Config::loadConfig("/home/trung/catkin_ws/src/thesis/config.json");
        } catch (const std::exception &e) {
            ROS_ERROR("Failed to load config.json: %s", e.what());
            return -1;
        }

        // Log camera settings
        CameraSettings cameraSettings = Config::config.camera;
        std::cout << "📷 Camera settings loaded:\n";
        std::cout << "   Focal Length: " << cameraSettings.focalLength << " mm\n";
        std::cout << "   Real Object Width: " << cameraSettings.realObjectWidth << " m\n";
        std::cout << "   FPS: " << cameraSettings.fps << " FPS\n";
        std::cout << "✅ Configuration loaded successfully.\n";

        // Initialize tracking
        try {
            STrack::initializeEstimator();
            ROS_INFO("✅ Tracking estimator initialized successfully");
        } catch (const std::exception &e) {
            ROS_ERROR("Failed to initialize tracking estimator: %s", e.what());
            return -1;
        }

        // Create enhanced video subscriber
        EnhancedVideoSubscriber subscriber;

        ROS_INFO("🚗 Enhanced Video Subscriber is ready!");
        ROS_INFO("📡 Subscribed to: /video/image");
        ROS_INFO("📊 Publishing topics:");
        ROS_INFO("   - /ego_speed (Float32)");
        ROS_INFO("   - /driving_action (String)");
        ROS_INFO("   - /cmd_vel (Twist)");

        // Start processing
        ros::spin();

    } catch (const std::exception &e) {
        ROS_ERROR("Exception in main: %s", e.what());
        return -1;
    }

    ROS_INFO("Enhanced Video Subscriber shutting down...");
    return 0;
}