#pragma once

#include <deque>
#include <opencv2/opencv.hpp>
#include <stdexcept>
#include <vector>

class LaneDetector {
   public:
    LaneDetector();
    ~LaneDetector() = default;

    // Main lane detection method
    std::vector<cv::Vec4i> detectLanes(const cv::Mat& frame);

    // Draw detected lanes
    void drawLanes(cv::Mat& frame, const std::vector<cv::Vec4i>& lines);

   private:
    // Configuration parameters
    struct Config {
        int canny_low = 50;
        int canny_high = 150;
        cv::Size blur_kernel = cv::Size(5, 5);
        int hough_rho = 1;
        double hough_theta = CV_PI / 180.0;
        int hough_threshold = 40;
        int min_line_length = 30;
        int max_line_gap = 25;
        cv::Scalar line_color = cv::Scalar(0, 255, 255);  // Yellow
        int line_thickness = 4;
        cv::Scalar fill_color = cv::Scalar(0, 255, 0);  // Green
        float fill_alpha = 0.3f;
        double slope_threshold = 0.4;
        int confidence_window = 5;
        float roi_bottom_width = 0.80f;
        float roi_top_width = 0.08f;
        float roi_height = 0.63f;
        float min_detection_confidence = 0.5f;
    } config_;

    // State management
    int width_ = 0;
    int height_ = 0;
    std::pair<double, double> prev_left_ = {0.0, 0.0};   // Slope, intercept
    std::pair<double, double> prev_right_ = {0.0, 0.0};  // Slope, intercept
    std::deque<std::pair<double, double>> left_history_;
    std::deque<std::pair<double, double>> right_history_;

    // Private methods
    cv::Mat preprocess(const cv::Mat& frame);
    std::vector<cv::Vec4i> houghTransform(const cv::Mat& edges);
    cv::Mat getRoiMask();
    std::pair<std::vector<std::pair<double, double>>, std::vector<std::pair<double, double>>>
    filterLines(const std::vector<cv::Vec4i>& lines);
    std::pair<std::pair<double, double>, double> calculateLane(
        const std::vector<std::pair<double, double>>& lines,
        std::deque<std::pair<double, double>>& history, const std::pair<double, double>& prev_lane);
    std::pair<std::pair<double, double>, double> robustLineFit(
        const std::vector<cv::Point2i>& points);
};