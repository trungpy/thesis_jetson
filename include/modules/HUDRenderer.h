#pragma once
#include <opencv2/opencv.hpp>
#include <string>

class HUDRenderer {
private:
    double avg_processing_time_ = 0;
    bool emergency_stop_ = false;

public:
    void setEmergencyStop(bool active);

    void render(cv::Mat &image, float ego_speed, int acc_speed,
                float front_speed, float avg_distance, bool accActive,
                const std::string &action_str, const cv::Scalar &action_color,
                double fps, int target_id);
};
