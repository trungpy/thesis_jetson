#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>

#include <opencv2/opencv.hpp>

class VideoPublisher {
   private:
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Publisher image_pub_;
    cv::VideoCapture cap_;
    ros::Timer timer_;

    std::string video_path_;
    double fps_;

   public:
    VideoPublisher() : it_(nh_) {
        // Get video path parameter (try both private and global)
        ros::NodeHandle private_nh("~");
        private_nh.param<std::string>("video_path", video_path_, "");
        private_nh.param<double>("fps", fps_, 30.0);

        // If not found in private, try global
        if (video_path_.empty()) {
            nh_.param<std::string>("video_path", video_path_, "");
        }

        if (video_path_.empty()) {
            ROS_ERROR("No video path specified! Use: _video_path:=/path/to/video.mp4");
            ros::shutdown();
            return;
        }

        // Open video file
        if (!cap_.open(video_path_)) {
            ROS_ERROR("Could not open video: %s", video_path_.c_str());
            ros::shutdown();
            return;
        }

        ROS_INFO("Publishing video: %s at %.1f fps", video_path_.c_str(), fps_);

        // Create publisher and timer
        image_pub_ = it_.advertise("video/image", 1);
        timer_ = nh_.createTimer(ros::Duration(1.0 / fps_), &VideoPublisher::publishFrame, this);
    }

    void publishFrame(const ros::TimerEvent&) {
        cv::Mat frame;

        if (!cap_.read(frame)) {
            // Loop back to start
            cap_.set(cv::CAP_PROP_POS_FRAMES, 0);
            if (!cap_.read(frame)) {
                ROS_ERROR("Failed to read frame");
                return;
            }
        }

        // Convert and publish
        sensor_msgs::ImagePtr msg =
            cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();
        msg->header.stamp = ros::Time::now();

        image_pub_.publish(msg);
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "video_publisher");

    VideoPublisher publisher;
    ros::spin();

    return 0;
}