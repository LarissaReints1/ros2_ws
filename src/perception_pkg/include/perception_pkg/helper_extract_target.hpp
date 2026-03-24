#pragma once

#include <opencv2/core.hpp>
#include <opencv2/dnn.hpp>
#include <string>
#include <vector>


struct DetectionResult {
    bool found;
    double u; // x pixel coordinate
    double v; // y pixel coordinate
};

enum class DetectionMode {
    RED_TARGET,
    DL_TARGET
};

class TargetDetector {
public:
    TargetDetector();

    // Set detection mode: "RED_TARGET" or "DL_TARGET"
    void setMode(const std::string &mode_str);

    // Detect a target (red or person depending on mode)
    DetectionResult detect(const cv::Mat &frame);

private:
    DetectionMode mode_;
    cv::dnn::Net net_;
    bool yolo_loaded_ = false;
   
    // Detection implementations
    DetectionResult detectRed(const cv::Mat &frame);
    DetectionResult detectDL(const cv::Mat &frame);

};
