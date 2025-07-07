#include "modules/VideoRecorder.h"
#include <chrono>
#include <iomanip>
#include <sstream>
#include <sys/stat.h>
#include <unistd.h>

void VideoRecorder::init(const cv::Mat &first_frame,
                         const std::string &output_dir) {
    if (initialized_)
        return;

    struct stat info;
    if (stat(output_dir.c_str(), &info) != 0) {
        mkdir(output_dir.c_str(), 0755);
    }

    auto now = std::chrono::system_clock::now();
    auto time_t = std::chrono::system_clock::to_time_t(now);
    std::stringstream ss;
    ss << output_dir << "/rec_"
       << std::put_time(std::localtime(&time_t), "%m-%d_%H-%M") << ".avi";
    filename_ = ss.str();

    int codec = cv::VideoWriter::fourcc('H', '2', '6', '4');
    double fps = 30.0;
    bool isColor = (first_frame.type() == CV_8UC3);
    writer_.open(filename_, codec, fps, first_frame.size(), isColor);

    if (writer_.isOpened()) {
        initialized_ = true;
    }
}

void VideoRecorder::writeFrame(const cv::Mat &frame) {
    if (initialized_ && writer_.isOpened()) {
        writer_ << frame;
    }
}

VideoRecorder::~VideoRecorder() {
    if (writer_.isOpened()) {
        writer_.release();
    }
}