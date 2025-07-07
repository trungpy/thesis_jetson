#pragma once
#include <opencv2/opencv.hpp>
#include <string>

class VideoRecorder {
private:
    cv::VideoWriter writer_;
    bool initialized_ = false;
    std::string filename_;

public:
    void init(const cv::Mat &first_frame, const std::string &output_dir);
    void writeFrame(const cv::Mat &frame);
    bool isInitialized() const { return initialized_; }
    ~VideoRecorder();
};
