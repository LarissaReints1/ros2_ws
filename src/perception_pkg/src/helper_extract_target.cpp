#include "perception_pkg/helper_extract_target.hpp"

#include <iostream>
#include <algorithm>
#include <opencv2/opencv.hpp>

// ---------------- Constructor ----------------
TargetDetector::TargetDetector()
: mode_(DetectionMode::RED_TARGET)
{
    std::cout << "[TargetDetector] Initialized in RED_TARGET mode\n";
    yolo_loaded_ = false;  // Explicitly disabled
}

// ---------------- Set modes ----------------
void TargetDetector::setMode(const std::string &mode_str)
{
    if (mode_str == "RED_TARGET") {
        mode_ = DetectionMode::RED_TARGET;
    } else {
        std::cerr << "[TargetDetector] Only RED_TARGET supported. Falling back.\n";
        mode_ = DetectionMode::RED_TARGET;
    }
}

// ---------------- Detect ----------------
DetectionResult TargetDetector::detect(const cv::Mat &frame)
{
    return detectRed(frame);
}

// ---------------- Red target detection ----------------
DetectionResult TargetDetector::detectRed(const cv::Mat &frame)
{
    if (frame.empty())
        return {false, -1, -1};

    // Convert to HSV
    cv::Mat hsv;
    cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);
    cv::GaussianBlur(hsv, hsv, cv::Size(5, 5), 0);

    // Red color ranges (two ranges because HSV wraps)
    cv::Mat mask1, mask2;
    cv::inRange(hsv,
                cv::Scalar(0, 120, 70),
                cv::Scalar(10, 255, 255),
                mask1);

    cv::inRange(hsv,
                cv::Scalar(160, 120, 70),
                cv::Scalar(179, 255, 255),
                mask2);

    cv::Mat mask = mask1 | mask2;

    // Morphology cleanup
    cv::erode(mask, mask, cv::Mat(), cv::Point(-1, -1), 2);
    cv::dilate(mask, mask, cv::Mat(), cv::Point(-1, -1), 2);

    // Find contours
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    if (contours.empty())
        return {false, -1, -1};

    // Largest contour = target
    auto largest = std::max_element(
        contours.begin(), contours.end(),
        [](const auto &a, const auto &b) {
            return cv::contourArea(a) < cv::contourArea(b);
        });

    if (cv::contourArea(*largest) < 150.0)
        return {false, -1, -1};

    // Compute centroid
    cv::Moments m = cv::moments(*largest);
    if (m.m00 <= 0.0)
        return {false, -1, -1};

    double cx = m.m10 / m.m00;
    double cy = m.m01 / m.m00;

    return {true, cx, cy};
}

// ---------------- DL stub (disabled) ----------------
DetectionResult TargetDetector::detectDL(const cv::Mat &)
{
    return {false, -1, -1};
}
