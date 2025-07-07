#include "modules/DataLogger.h"
#include <chrono>
#include <iomanip>
#include <ros/ros.h>
#include <sstream>
#include <sys/stat.h>
#include <unistd.h>

void DataLogger::init(const std::string &output_dir) {
    struct stat info;
    if (stat(output_dir.c_str(), &info) != 0) {
        mkdir(output_dir.c_str(), 0755);
    }

    auto now = std::chrono::system_clock::now();
    auto time_t = std::chrono::system_clock::to_time_t(now);
    std::stringstream ss;
    ss << output_dir << "/log_"
       << std::put_time(std::localtime(&time_t), "%m-%d_%H-%M") << ".csv";
    log_filename_ = ss.str();

    log_file_.open(log_filename_);
    if (log_file_.is_open()) {
        log_file_ << "timestamp,frame_count,fps,ego_speed,target_id,action,"
                  << "detection_count,avg_distance,front_speed,processing_time,"
                     "max_speed,acc_speed\n";
        ROS_INFO("Data logging initialized: %s", log_filename_.c_str());
    } else {
        ROS_ERROR("Failed to open log file: %s", log_filename_.c_str());
    }
}

void DataLogger::log(double timestamp, int frame_count, double fps,
                     float ego_speed, int target_id, const std::string &action,
                     int detection_count, float avg_distance, float front_speed,
                     double processing_time, int max_speed, int acc_speed) {
    if (!log_file_.is_open())
        return;

    log_file_ << std::fixed << std::setprecision(3) << timestamp << ","
              << frame_count << "," << fps << "," << ego_speed << ","
              << target_id << "," << action << "," << detection_count << ","
              << avg_distance << "," << front_speed << "," << processing_time
              << "," << max_speed << "," << acc_speed << "\n";
    log_file_.flush();
}

DataLogger::~DataLogger() {
    if (log_file_.is_open()) {
        log_file_.close();
    }
}
