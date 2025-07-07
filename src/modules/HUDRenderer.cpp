#include "modules/HUDRenderer.h"

void HUDRenderer::setEmergencyStop(bool active) { emergency_stop_ = active; }

void HUDRenderer::render(cv::Mat &image, float ego_speed, int acc_speed,
                         float front_speed, float avg_distance, bool accActive,
                         const std::string &action_str,
                         const cv::Scalar &action_color, double fps,
                         int target_id,
                         float engine_force, float throttle_force, float brake_force) {
    int font = cv::FONT_HERSHEY_SIMPLEX;
    double font_scale = 0.6;
    int thickness = 2;

    cv::rectangle(image, cv::Point(10, 10), cv::Point(420, 180),
                  cv::Scalar(0, 0, 0, 128), -1);
    cv::putText(image, "Speed Control:", cv::Point(20, 35), font, font_scale,
                cv::Scalar(255, 255, 255), thickness);
    cv::putText(image, "Ego: " + std::to_string((int)ego_speed) + " km/h",
                cv::Point(20, 55), font, font_scale, cv::Scalar(0, 255, 0),
                thickness);
    cv::putText(image, "Target: " + std::to_string(acc_speed) + " km/h",
                cv::Point(20, 75), font, font_scale, cv::Scalar(255, 255, 0),
                thickness);

    cv::putText(image, "Action: " + action_str, cv::Point(20, 125), font,
                font_scale, action_color, thickness);
    cv::putText(image,
                "Cruise active: " +
                    std::string(accActive ? "Enable" : "Disable"),
                cv::Point(120, 100), font, font_scale,
                cv::Scalar(255, 20, 20), thickness);

    cv::putText(image, "Target ID: " + std::to_string(target_id),
                cv::Point(20, 145), font, font_scale, cv::Scalar(255, 255, 255),
                thickness);

    cv::putText(image, "FPS: " + std::to_string((int)fps), cv::Point(20, 165),
                font, font_scale, cv::Scalar(255, 255, 255), thickness);

    // --- Engine/Throttle/Brake force info ---
    cv::putText(image, "Engine: " + std::to_string((int)engine_force) + " N",
                cv::Point(200, 125), font, font_scale, cv::Scalar(255, 200, 0), thickness);
    cv::putText(image, "Throttle: " + std::to_string((int)throttle_force) + " N",
                cv::Point(200, 145), font, font_scale, cv::Scalar(0, 255, 255), thickness);
    cv::putText(image, "Brake: " + std::to_string((int)brake_force) + " N",
                cv::Point(200, 165), font, font_scale, cv::Scalar(0, 128, 255), thickness);

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
                    cv::Point(200, 55), font, font_scale,
                    cv::Scalar(255, 255, 255), thickness);
    }

    if (front_speed > 0) {
        cv::putText(
            image, "Front Speed: " + std::to_string((int)front_speed) + " km/h",
            cv::Point(200, 75), font, font_scale, cv::Scalar(255, 255, 255),
            thickness);
    }
}
