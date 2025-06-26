#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <image_transport/image_transport.h>
#include <chrono>
#include <iostream>
#include <string>
#include <vector>

#include <sys/stat.h>
#include <unistd.h>

#include "Detect.h"
#include <lanevision/LaneDetector.h>

using namespace std;

const int WIDTH = 1280;
const int HEIGHT = 720;
const int FPS = 30;

bool IsPathExist(const string &path)
{

    return (access(path.c_str(), F_OK) == 0);
}

bool IsFile(const string &path)
{
    if (!IsPathExist(path))
    {
        printf("%s:%d %s not exist\n", __FILE__, __LINE__, path.c_str());
        return false;
    }

    struct stat buffer;
    return (stat(path.c_str(), &buffer) == 0 && S_ISREG(buffer.st_mode));
}

/**
 * @brief Setting up Tensorrt logger
 */
class Logger : public nvinfer1::ILogger
{
    void log(Severity severity, const char *msg) noexcept override
    {
        // Only output logs with severity greater than warning
        if (severity <= Severity::kWARNING)
            std::cout << msg << std::endl;
    }
} logger;

class VideoSubscriber
{
private:
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;

    // ROS frame counting
    int frame_count_;
    ros::Time last_time_;

    // Processing variables
    double fps_;
    int frameCount_;
    int frameCountSave_;
    std::chrono::steady_clock::time_point fpsStartTime_;

    // Speed control variables
    int maxSpeed_;    // km/h
    int accMaxSpeed_; // km/h
    int accSpeed_;    // km/h
    bool saveImage_;

    std::string model_path_;

    // Detection models
    Detect model_;
    LaneDetector laneDetector_;
    std::string getModelPath()
    {
        ros::NodeHandle private_nh("~");
        private_nh.param<std::string>("model_path", model_path_, "");
        // If not found in private, try global
        if (model_path_.empty())
        {
            nh_.param<std::string>("video_path", model_path_, "");
        }
        if (model_path_.empty())
        {
            ROS_ERROR("No model path specified! Use: model_path_:=/path/to/model.engine");
            ros::shutdown();
            return "";
        }
        return model_path_;
    }

public:
    VideoSubscriber() : it_(nh_),
                        frame_count_(0),
                        fps_(0.0),
                        frameCount_(0),
                        frameCountSave_(0),
                        maxSpeed_(-1),
                        accMaxSpeed_(90),
                        accSpeed_(60),
                        saveImage_(false),
                        model_(getModelPath(), logger), // Initialize in member init list
                        laneDetector_()
    {
        // Subscribe to video topic
        image_sub_ = it_.subscribe("video/image", 1,
                                   &VideoSubscriber::imageCallback, this);

        last_time_ = ros::Time::now();
        fpsStartTime_ = std::chrono::steady_clock::now();

        ROS_INFO("Video subscriber started. Waiting for frames...");
        ROS_INFO("Model loaded: %s", model_path_.c_str());
    }

    void imageCallback(const sensor_msgs::Image::ConstPtr &msg)
    {
        try
        {
            // Convert ROS image to OpenCV
            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg,
                                                               sensor_msgs::image_encodings::BGR8);

            auto now = std::chrono::steady_clock::now();

            if (cv_ptr->image.empty())
            {
                ROS_WARN("Empty image received");
                return;
            }

            // Create a copy for processing
            cv::Mat image = cv_ptr->image.clone();

            // Object detection
            vector<Detection> objects;
            const float ratio_h = model_.getInputH() / (float)image.rows;
            const float ratio_w = model_.getInputW() / (float)image.cols;

            model_.preprocess(image);
            model_.infer();
            model_.postprocess(objects);

            // Lane detection
            std::vector<cv::Vec4i> lanes = laneDetector_.detectLanes(image);

            // Process detected objects
            processDetections(image, objects, lanes);

            // Draw results
            model_.draw(image, objects);
            laneDetector_.drawLanes(image, lanes);

            // Calculate and display FPS
            calculateFPS(now);
            drawOverlays(image);

            // Save image if needed
            if (saveImage_)
            {
                saveFrame(image);
                saveImage_ = false;
            }

            // Display the processed frame
            cv::imshow("Processed Video", image);
            cv::waitKey(1);
        }
        catch (cv_bridge::Exception &e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
        }
        catch (const std::exception &e)
        {
            ROS_ERROR("Processing exception: %s", e.what());
        }
    }

private:
    void processDetections(cv::Mat &image, const vector<Detection> &objects, const std::vector<cv::Vec4i> &lanes)
    {
        const float ratio_h = model_.getInputH() / (float)image.rows;
        const float ratio_w = model_.getInputW() / (float)image.cols;

        for (const auto &obj : objects)
        {
            auto box = obj.bbox;
            auto class_id = obj.class_id;
            auto conf = obj.conf;

            // Adjust box back to original image size
            if (ratio_h > ratio_w)
            {
                box.x = box.x / ratio_w;
                box.y = (box.y - (model_.getInputH() - ratio_w * image.rows) / 2) / ratio_w;
                box.width = static_cast<int>(box.width / ratio_w);
                box.height = static_cast<int>(box.height / ratio_w);
            }
            else
            {
                box.x = (box.x - (model_.getInputW() - ratio_h * image.cols) / 2) / ratio_h;
                box.y = box.y / ratio_h;
                box.width = static_cast<int>(box.width / ratio_h);
                box.height = static_cast<int>(box.height / ratio_h);
            }

            // Clamp box coordinates to image size
            box.x = std::max(0.0f, static_cast<float>(box.x));
            box.y = std::max(0.0f, static_cast<float>(box.y));
            box.width = std::min(static_cast<float>(box.width), static_cast<float>(image.cols - box.x));
            box.height = std::min(static_cast<float>(box.height), static_cast<float>(image.rows - box.y));

            cv::Point bottom_center(box.x + box.width / 2, box.y + box.height);

            // Check if vehicle is in lane
            if (lanes.size() >= 2)
            {
                checkVehicleInLane(image, lanes, bottom_center, obj);
            }

            // Check for speed limit signs
            if (class_id >= 12 && class_id <= 17 && conf > 0.9)
            {
                int newSpeed = (class_id - 9) * 10; // class_id 12 -> 30km/h, 13 -> 40, etc.
                maxSpeed_ = newSpeed;
                saveImage_ = true;
                ROS_INFO("Class_id: %d", class_id);
                ROS_INFO("Speed limit detected: %d km/h", newSpeed);
            }
        }
    }

    void checkVehicleInLane(cv::Mat &image, const std::vector<cv::Vec4i> &lanes,
                            const cv::Point &bottom_center, const Detection &obj)
    {
        // Get 2 lanes (left - right)
        cv::Vec4i l0 = lanes[0];
        cv::Vec4i l1 = lanes[1];

        // Create a polygon for the lane area
        std::vector<cv::Point> lane_area = {
            cv::Point(l0[0], l0[1]), // top-left
            cv::Point(l1[0], l1[1]), // top-right
            cv::Point(l1[2], l1[3]), // bottom-right
            cv::Point(l0[2], l0[3])  // bottom-left
        };

        // Only consider car/bus/truck
        if (obj.class_id == 2 || obj.class_id == 4 || obj.class_id == 5)
        {
            if (cv::pointPolygonTest(lane_area, bottom_center, false) >= 0)
            {
                // Vehicle is in the lane - draw green dot
                cv::Point mid(obj.bbox.x + obj.bbox.width / 2,
                              obj.bbox.y + obj.bbox.height / 2);
                cv::circle(image, mid, 5, cv::Scalar(0, 255, 0), -1);
            }
        }
    }

    void calculateFPS(const std::chrono::steady_clock::time_point &now)
    {
        frameCount_++;
        auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(now - fpsStartTime_).count();

        if (elapsed >= 1)
        {
            fps_ = frameCount_ / static_cast<double>(elapsed);
            frameCount_ = 0;
            fpsStartTime_ = now;
        }
    }

    void drawOverlays(cv::Mat &image)
    {
        // Draw FPS
        cv::putText(image, cv::format("FPS: %.2f", fps_), cv::Point(10, 30),
                    cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 0), 2);

        // Draw max speed
        if (maxSpeed_ != -1)
        {
            cv::putText(image, cv::format("Max Speed: %dKm/h", maxSpeed_), cv::Point(10, 60),
                        cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(255, 0, 0), 2);
        }
        else
        {
            cv::putText(image, "No Speed Limit Detected", cv::Point(10, 60),
                        cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(255, 0, 0), 2);
        }

        // Draw Cruise Control
        cv::putText(image, cv::format("Max Cruise Control: %dKm/h", accMaxSpeed_), cv::Point(10, 90),
                    cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 255), 2);

        // Draw control speed
        cv::putText(image, cv::format("Control speed: %dKm/h", accSpeed_), cv::Point(10, 120),
                    cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 0, 255), 2);

        // Update speed control logic
        updateSpeedControl();
    }

    void updateSpeedControl()
    {
        if (maxSpeed_ != -1)
        {
            if (accSpeed_ < maxSpeed_ && accSpeed_ < accMaxSpeed_)
            {
                accSpeed_ += 1; // Increase speed by 1 km/h
            }
            else if (accSpeed_ > maxSpeed_ && accSpeed_ > 0)
            {
                accSpeed_ -= 1; // Decrease speed by 1 km/h
            }
        }
        else
        {
            accSpeed_ = accMaxSpeed_; // Reset to max speed if no speed limit detected
        }
    }

    void saveFrame(const cv::Mat &image)
    {
        std::vector<int> params = {cv::IMWRITE_JPEG_QUALITY, 95};
        std::string filename = std::to_string(frameCountSave_) + ".jpg";
        cv::imwrite(filename, image, params);
        frameCountSave_++;
        ROS_INFO("Saved frame: %s", filename.c_str());
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "video_subscriber_test");

    try
    {
        VideoSubscriber subscriber;

        ROS_INFO("Advanced video subscriber test running...");
        ROS_INFO("Subscribing to: /video/image");
        ROS_INFO("Features: Object detection, Lane detection, Speed control");

        ros::spin();
    }
    catch (const std::exception &e)
    {
        ROS_ERROR("Exception in main: %s", e.what());
        return -1;
    }

    cv::destroyAllWindows();
    return 0;
}