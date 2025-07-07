#include "modules/HUDRenderer.h"

void HUDRenderer::setEmergencyStop(bool active) { emergency_stop_ = active; }

void HUDRenderer::render(cv::Mat &image, float ego_speed, int acc_speed,
                         float front_speed, float avg_distance, bool accActive,
                         const std::string &action_str,
                         const cv::Scalar &action_color, double fps,
                         int target_id) {
    int font = cv::FONT_HERSHEY_SIMPLEX;
    double font_scale = 0.6;
    int thickness = 2;

    cv::rectangle(image, cv::Point(10, 10), cv::Point(420, 200),
                  cv::Scalar(0, 0, 0, 128), -1);
    cv::putText(image, "Enhanced Driving Assistant", cv::Point(20, 35), font,
                0.7, cv::Scalar(255, 255, 255), 2);

    cv::putText(image, "Speed Control:", cv::Point(20, 65), font, font_scale,
                cv::Scalar(255, 255, 255), thickness);
    cv::putText(image, "Ego: " + std::to_string((int)ego_speed) + " km/h",
                cv::Point(20, 85), font, font_scale, cv::Scalar(0, 255, 0),
                thickness);
    cv::putText(image, "Target: " + std::to_string(acc_speed) + " km/h",
                cv::Point(20, 105), font, font_scale, cv::Scalar(255, 255, 0),
                thickness);

    cv::putText(image, "Action: " + action_str, cv::Point(20, 155), font,
                font_scale, action_color, thickness);
    cv::putText(image,
                "Cruise active: " +
                    std::string(accActive ? "Enable" : "Disable"),
                cv::Point(200, 155), font, font_scale,
                cv::Scalar(255, 255, 255), thickness);

    cv::putText(image, "Target ID: " + std::to_string(target_id),
                cv::Point(20, 175), font, font_scale, cv::Scalar(255, 255, 255),
                thickness);

    cv::putText(image, "FPS: " + std::to_string((int)fps), cv::Point(20, 195),
                font, font_scale, cv::Scalar(255, 255, 255), thickness);

    if (emergency_stop_) {
        cv::rectangle(image, cv::Point(image.cols - 150, 10),
                      cv::Point(image.cols - 10, 50), cv::Scalar(0, 0, 255),
                      -1);
        cv::putText(image, "EMERGENCY STOP", cv::Point(image.cols - 145, 35),
                    font, 0.5, cv::Scalar(255, 255, 255), 2);
    }

    if (avg_distance > 0) {
        cv::putText(image,
                    "Distance: " + std::to_string((int)avg_distance) + "m",
                    cv::Point(200, 85), font, font_scale,
                    cv::Scalar(255, 255, 255), thickness);
    }

    if (front_speed > 0) {
        cv::putText(
            image, "Front Speed: " + std::to_string((int)front_speed) + " km/h",
            cv::Point(200, 105), font, font_scale, cv::Scalar(255, 255, 255),
            thickness);
    }
}
