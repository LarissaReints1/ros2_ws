#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "camera_info_manager/camera_info_manager.hpp"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/opencv.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"
#include <filesystem>
#include <string>

using namespace std::chrono_literals;

class Cam2ImageNode : public rclcpp::Node
{
public:
    Cam2ImageNode() : Node("cam2image_node"), resolution_printed_(false)
    {
        // --- Parameters ---
        this->declare_parameter<bool>("camera_stream", false);
        this->declare_parameter<std::string>("stream_url", "0");
        this->declare_parameter<std::string>("frame_id", "camera_link");
        this->declare_parameter<std::string>("camera_info_file", "camera_info_highres.yaml");
        this->declare_parameter<std::string>("camera_name", "rx0_camera");
        this->declare_parameter<bool>("flip_vertical", false);
        this->declare_parameter<bool>("flip_horizontal", false);
        this->declare_parameter<int>("fps", 30);
        this->declare_parameter<int>("width", 640);
        this->declare_parameter<int>("height", 480);

        // --- Get parameters ---
        this->get_parameter("stream_url", stream_url_);
        this->get_parameter("camera_name", camera_name_);
        this->get_parameter("frame_id", frame_id_);
        this->get_parameter("camera_info_file", camera_info_file_);
        this->get_parameter("flip_vertical", flip_vertical_);
        this->get_parameter("flip_horizontal", flip_horizontal_);
        this->get_parameter("fps", fps_);
        this->get_parameter("camera_stream", camera_stream_);
        this->get_parameter("width", width_);
        this->get_parameter("height", height_);

        // --- Path to camera_info YAML ---
        auto package_share = ament_index_cpp::get_package_share_directory("camera_pkg");
        std::filesystem::path yaml_path = std::filesystem::path(package_share) / "config" / camera_info_file_;
        camera_info_url_ = "file://" + yaml_path.string();

        RCLCPP_INFO_STREAM(this->get_logger(),
                           "Camera: " << camera_name_
                                      << " | Frame ID: " << frame_id_
                                      << " | Resolution: " << width_ << "x" << height_
                                      << " | FPS: " << fps_
                                      << " | Camera stream: " << (camera_stream_ ? "true" : "false")
                                      << " | Camera info: " << camera_info_url_);

        // --- Publishers ---
        publisher_ = this->create_publisher<sensor_msgs::msg::Image>("camera/image", 10);
        camera_info_pub_ = this->create_publisher<sensor_msgs::msg::CameraInfo>(
            "camera/camera_info", rclcpp::QoS(1).transient_local());

        // --- Open camera stream ---
        if (camera_stream_)
        {
            RCLCPP_INFO(this->get_logger(), "Opening camera stream: %s", stream_url_.c_str());
            cap_.open(stream_url_);
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "Opening USB camera using GStreamer");

            std::string device = "/dev/video" + stream_url_;

            // ---- GStreamer pipeline ---
            std::string pipeline =
                "v4l2src device=" + device + " ! "
                                             "video/x-raw, width=" +
                std::to_string(width_) +
                ", height=" + std::to_string(height_) +
                ", framerate=" + std::to_string(fps_) + "/1 ! "
                                                        "nvvidconv ! "
                                                        "video/x-raw, format=BGRx ! "
                                                        "videoconvert ! "
                                                        "video/x-raw, format=BGR ! "
                                                        "appsink";

            RCLCPP_INFO_STREAM(this->get_logger(), "Pipeline: " << pipeline);

            cap_.open(pipeline, cv::CAP_GSTREAMER);
        }

        if (!cap_.isOpened())
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to open camera source");
            throw std::runtime_error("Failed to open camera source");
        }

        // --- Camera Info Manager ---
        cam_info_mgr_ = std::make_shared<camera_info_manager::CameraInfoManager>(
            this, camera_name_, camera_info_url_);

        if (!cam_info_mgr_->isCalibrated())
        {
            RCLCPP_WARN(this->get_logger(), "Camera not calibrated, publishing uncalibrated info.");
        }

        // --- Timer ---
        auto period = std::chrono::milliseconds(1000 / fps_);
        timer_ = this->create_wall_timer(period, std::bind(&Cam2ImageNode::timer_callback, this));

        RCLCPP_INFO(this->get_logger(), "Camera node started");
    }

private:
    void timer_callback()
    {
        cv::Mat frame;
        cap_ >> frame;

        if (frame.empty())
        {
            RCLCPP_WARN(this->get_logger(), "Empty frame captured");
            return;
        }

        // --- Apply flips if needed ---
        if (flip_vertical_ && flip_horizontal_)
            cv::flip(frame, frame, -1);
        else if (flip_vertical_)
            cv::flip(frame, frame, 0);
        else if (flip_horizontal_)
            cv::flip(frame, frame, 1);

        // --- Publish ROS Image Msgs ---
        auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame).toImageMsg();
        msg->header.stamp = this->now();
        msg->header.frame_id = frame_id_;
        publisher_->publish(*msg);

        // --- Publish Camera Info Msgs ---
        sensor_msgs::msg::CameraInfo info = cam_info_mgr_->getCameraInfo();
        info.header.stamp = msg->header.stamp;
        info.header.frame_id = frame_id_;
        camera_info_pub_->publish(info);
    }

    // --- Members ---
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_pub_;
    cv::VideoCapture cap_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::string stream_url_;
    std::string camera_name_;
    std::string frame_id_;
    std::string camera_info_file_;
    std::string camera_info_url_;
    bool flip_vertical_;
    bool flip_horizontal_;
    int fps_;
    bool camera_stream_;
    int width_;
    int height_;

    std::shared_ptr<camera_info_manager::CameraInfoManager> cam_info_mgr_;
    bool resolution_printed_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Cam2ImageNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}