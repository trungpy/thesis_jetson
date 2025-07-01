// =============================================================================
// Project: LaneVision
// Description: LaneVision is a C++ library for detecting and tracking road lanes in images.
// License: MIT License
// =============================================================================
// Copyright (c) 2025 [Your Name or Organization]
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
// =============================================================================

#include "LaneDetector.h"

#include <algorithm>
#include <numeric>

LaneDetector::LaneDetector()
    : left_history_(config_.confidence_window), right_history_(config_.confidence_window) {}

std::vector<cv::Vec4i> LaneDetector::detectLanes(const cv::Mat &frame) {
    if (frame.empty()) {
        throw std::invalid_argument("Input frame is empty");
    }
    height_ = frame.rows;
    width_ = frame.cols;
    cv::Mat edges = preprocess(frame);

    std::vector<cv::Vec4i> raw_lines = houghTransform(edges);
    cv::imshow("edge", edges);

    // Filter and classify lines
    auto [left_lines, right_lines] = filterLines(raw_lines);

    // Calculate lanes with temporal smoothing
    auto [left_lane, left_conf] = calculateLane(left_lines, left_history_, prev_left_);
    auto [right_lane, right_conf] = calculateLane(right_lines, right_history_, prev_right_);

    // Update previous lanes
    if (left_conf > 0.0) {
        prev_left_ = left_lane;
    }
    if (right_conf > 0.0) {
        prev_right_ = right_lane;
    }

    // Convert lanes to Vec4i for output
    std::vector<cv::Vec4i> lane_lines;
    int y1 = static_cast<int>(height_ * config_.roi_height);
    int y2 = height_;
    auto clamp_x = [this](int x) { return std::max(0, std::min(x, width_ - 1)); };

    if (left_conf > 0.0) {
        double slope = left_lane.first;
        double intercept = left_lane.second;
        int x1 = (slope != 0.0) ? clamp_x(static_cast<int>((y1 - intercept) / slope)) : 0;
        int x2 = (slope != 0.0) ? clamp_x(static_cast<int>((y2 - intercept) / slope)) : 0;
        lane_lines.emplace_back(x1, y1, x2, y2);
    }
    if (right_conf > 0.0) {
        double slope = right_lane.first;
        double intercept = right_lane.second;
        int x1 = (slope != 0.0) ? clamp_x(static_cast<int>((y1 - intercept) / slope)) : 0;
        int x2 = (slope != 0.0) ? clamp_x(static_cast<int>((y2 - intercept) / slope)) : 0;
        lane_lines.emplace_back(x1, y1, x2, y2);
    }

    return lane_lines;
}

void LaneDetector::drawLanes(cv::Mat &frame, const std::vector<cv::Vec4i> &lines) {
    if (frame.empty()) {
        return;
    }

    cv::Mat overlay = frame.clone();
    int y1 = static_cast<int>(height_ * config_.roi_height);
    int y2 = height_;

    // Draw lanes
    if (lines.size() >= 2) {  // Assume first is left, second is right
        std::vector<cv::Point> pts;
        for (const auto &line : lines) {
            int x1 = line[0], y1 = line[1], x2 = line[2], y2 = line[3];
            cv::line(overlay, cv::Point(x1, y1), cv::Point(x2, y2), config_.line_color,
                     config_.line_thickness);
            pts.emplace_back(x1, y1);
            pts.emplace_back(x2, y2);
        }

        // Draw filled polygon
        if (pts.size() == 4) {
            std::vector<cv::Point> poly = {pts[0], pts[1], pts[3],
                                           pts[2]};  // Reorder for correct polygon
            cv::fillPoly(overlay, std::vector<std::vector<cv::Point>>{poly}, config_.fill_color);
        }
    }

    // Blend overlay with original frame
    cv::addWeighted(overlay, config_.fill_alpha, frame, 1.0f - config_.fill_alpha, 0.0, frame);
}

cv::Mat LaneDetector::preprocess(const cv::Mat &frame) {
    cv::Mat gray, blur, edges;
    cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
    cv::GaussianBlur(gray, blur, config_.blur_kernel, 0);
    cv::Canny(blur, edges, config_.canny_low, config_.canny_high);
    cv::Mat roi_mask = getRoiMask();
    cv::Mat masked_edges;
    cv::bitwise_and(edges, edges, masked_edges, roi_mask);
    return masked_edges;
}

std::vector<cv::Vec4i> LaneDetector::houghTransform(const cv::Mat &edges) {
    std::vector<cv::Vec4i> lines;
    cv::HoughLinesP(edges, lines, config_.hough_rho, config_.hough_theta, config_.hough_threshold,
                    config_.min_line_length, config_.max_line_gap);
    return lines;
}

cv::Mat LaneDetector::getRoiMask() {
    cv::Mat mask = cv::Mat::zeros(height_, width_, CV_8U);
    std::vector<cv::Point> vertices = {
        {static_cast<int>(width_ * (1 - config_.roi_bottom_width) / 2), height_},
        {static_cast<int>(width_ * (1 + config_.roi_bottom_width) / 2), height_},
        {static_cast<int>(width_ * (1 + config_.roi_top_width) / 2),
         static_cast<int>(height_ * config_.roi_height)},
        {static_cast<int>(width_ * (1 - config_.roi_top_width) / 2),
         static_cast<int>(height_ * config_.roi_height)}};
    cv::fillPoly(mask, std::vector<std::vector<cv::Point>>{vertices}, cv::Scalar(255));
    return mask;
}

std::pair<std::vector<std::pair<double, double>>, std::vector<std::pair<double, double>>>
LaneDetector::filterLines(const std::vector<cv::Vec4i> &lines) {
    std::vector<std::pair<double, double>> left_lines, right_lines;
    for (const auto &line : lines) {
        int x1 = line[0], y1 = line[1], x2 = line[2], y2 = line[3];
        double length = std::sqrt(std::pow(x2 - x1, 2) + std::pow(y2 - y1, 2));
        if (length < config_.min_line_length) {
            continue;
        }
        if (x1 == x2) {
            continue;  // Skip vertical lines
        }
        double slope = static_cast<double>(y2 - y1) / (x2 - x1);
        if (std::abs(slope) < config_.slope_threshold) {
            continue;
        }
        double intercept = y1 - slope * x1;
        if (slope < 0) {
            left_lines.emplace_back(slope, intercept);
        } else {
            right_lines.emplace_back(slope, intercept);
        }
    }
    return {left_lines, right_lines};
}

std::pair<std::pair<double, double>, double> LaneDetector::calculateLane(
    const std::vector<std::pair<double, double>> &lines,
    std::deque<std::pair<double, double>> &history, const std::pair<double, double> &prev_lane) {
    if (lines.empty()) {
        return {prev_lane, prev_lane.first != 0.0 ? 0.3 : 0.0};
    }

    std::vector<cv::Point2i> points;
    for (const auto &[slope, intercept] : lines) {
        int y1 = static_cast<int>(height_ * config_.roi_height);
        int y2 = height_;
        int x1 = (std::abs(slope) > 1e-6) ? static_cast<int>((y1 - intercept) / slope) : 0;
        int x2 = (std::abs(slope) > 1e-6) ? static_cast<int>((y2 - intercept) / slope) : 0;
        points.emplace_back(x1, y1);
        points.emplace_back(x2, y2);
    }

    if (points.size() < 2) {
        return {prev_lane, prev_lane.first != 0.0 ? 0.2 : 0.0};
    }

    auto [line, confidence] = robustLineFit(points);
    if (confidence < config_.min_detection_confidence) {
        return {prev_lane, prev_lane.first != 0.0 ? 0.2 : 0.0};
    }

    // Add to history and cap its size
    history.push_back(line);
    if (history.size() > static_cast<size_t>(config_.confidence_window)) {
        history.pop_front();
    }
    if (history.size() > 1) {
        double avg_slope =
            std::accumulate(history.begin(), history.end(), 0.0,
                            [](double sum, const auto &p) { return sum + p.first; }) /
            history.size();
        double avg_intercept =
            std::accumulate(history.begin(), history.end(), 0.0,
                            [](double sum, const auto &p) { return sum + p.second; }) /
            history.size();
        line = {avg_slope, avg_intercept};
        confidence = std::min(1.0, confidence * (1.0 + history.size() / 10.0));
    }

    return {line, confidence};
}

std::pair<std::pair<double, double>, double> LaneDetector::robustLineFit(
    const std::vector<cv::Point2i> &points) {
    if (points.size() < 2) {
        return {{0.0, 0.0}, 0.0};
    }

    // Compute mean and covariance for linear regression
    double x_mean = 0.0, y_mean = 0.0;
    for (const auto &p : points) {
        x_mean += p.x;
        y_mean += p.y;
    }
    x_mean /= points.size();
    y_mean /= points.size();

    double cov_xy = 0.0, var_x = 0.0;
    for (const auto &p : points) {
        double dx = p.x - x_mean;
        double dy = p.y - y_mean;
        cov_xy += dx * dy;
        var_x += dx * dx;
    }

    if (std::abs(var_x) < 1e-6) {  // Near-vertical line
        return {{std::numeric_limits<double>::infinity(), x_mean}, 1.0};
    }

    double slope = cov_xy / var_x;
    double intercept = y_mean - slope * x_mean;

    // Compute R-squared (confidence)
    double ss_tot = 0.0, ss_res = 0.0;
    for (const auto &p : points) {
        double y_pred = slope * p.x + intercept;
        ss_tot += std::pow(p.y - y_mean, 2);
        ss_res += std::pow(p.y - y_pred, 2);
    }
    double r_squared = (ss_tot > 1e-6) ? 1.0 - ss_res / ss_tot : 0.0;

    return {{slope, intercept}, r_squared};
}