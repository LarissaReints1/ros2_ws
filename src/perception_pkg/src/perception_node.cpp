#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"

#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include "cv_bridge/cv_bridge.h"
#include "opencv2/opencv.hpp"
#include "opencv2/dnn.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"

#include <filesystem>
#include <deque>
#include <vector>
#include <utility>
#include <algorithm>
#include <cmath>
#include <chrono>
#include <string>

#include "perception_pkg/helper_extract_target.hpp"

class PerceptionNode : public rclcpp::Node
{
public:
    PerceptionNode() : Node("perception_node")
    {
        // ---------------- Parameters ----------------
        this->declare_parameter<std::string>("input_topic", "/camera/image_raw");
        this->declare_parameter<bool>("use_dummy_target");
        this->declare_parameter<bool>("activate_filter", false);
        this->declare_parameter<int>("filter_size", 5);
        this->declare_parameter<bool>("activate_depth", false);
        this->declare_parameter<double>("depth_fps", 5.0);
        this->declare_parameter<std::string>("detection_mode", "RED_TARGET");
        this->declare_parameter<std::string>("external_depth_topic", "/camera/depth");
        this->declare_parameter<bool>("use_external_depth", true);
        this->declare_parameter<double>("depth_max_range", 5.0);

        this->get_parameter("input_topic", input_topic_);
        this->get_parameter("use_dummy_target", use_dummy_target_);
        this->get_parameter("activate_filter", activate_filter_);
        this->get_parameter("filter_size", filter_size_);
        this->get_parameter("activate_depth", activate_depth_);
        this->get_parameter("depth_fps", depth_fps_);
        this->get_parameter("detection_mode", mode_str_);
        this->get_parameter("external_depth_topic", external_depth_topic_);
        this->get_parameter("use_external_depth", use_external_depth_);
        this->get_parameter("depth_max_range", depth_max_range_);

        // -------------- SELECT MODE TARGETING --------

        detector_.setMode(mode_str_); // <- string works directly
        RCLCPP_INFO(this->get_logger(), "use_dummy_target: %s", use_dummy_target_ ? "true" : "false");

        // ---------------- TF2 ----------------
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // ---------------- Publishers ----------------
        publisher_ = this->create_publisher<sensor_msgs::msg::Image>("/perception/image", 10);
        pose_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/perception/pose", 10);
        target_pub_ = this->create_publisher<geometry_msgs::msg::PointStamped>("/perception/target_pixel", 10);
        depth_publisher_ = this->create_publisher<sensor_msgs::msg::Image>("/perception/depth_map", 10);

        // Subscribe to camera_info
        camera_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>("/camera/camera_info", 10,
                                                                                   std::bind(&PerceptionNode::camera_info_callback, this, std::placeholders::_1));

        depth_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            external_depth_topic_, 10,
            std::bind(&PerceptionNode::depth_callback, this, std::placeholders::_1));

        if (activate_depth_)
        {
            try
            {
                std::string pkg_path = ament_index_cpp::get_package_share_directory("perception_pkg");
                std::string model_path = pkg_path + "/models/model-small.onnx";

                // Check if file physically exists on disk
                if (!std::filesystem::exists(model_path))
                {
                    RCLCPP_ERROR(this->get_logger(), "FILE NOT FOUND at: %s", model_path.c_str());
                    return;
                }

                depth_net_ = cv::dnn::readNetFromONNX(model_path);

                // Recommended for CPU performance
                depth_net_.setPreferableBackend(cv::dnn::DNN_BACKEND_OPENCV);
                depth_net_.setPreferableTarget(cv::dnn::DNN_TARGET_CPU);

                depth_initialized_ = true;
                RCLCPP_INFO(this->get_logger(), "MiDaS model loaded successfully!");
            }
            catch (const std::exception &e)
            {
                RCLCPP_ERROR(this->get_logger(), "OpenCV DNN Error: %s", e.what());
            }
        }
        // ---------------- Mode Selection ----------------
        if (use_dummy_target_)
        {
            timer_ = this->create_wall_timer(
                std::chrono::milliseconds(100),
                std::bind(&PerceptionNode::publish_dummy_image, this));
            RCLCPP_INFO(this->get_logger(), "Mode: Dummy target");
        }
        else
        {
            subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
                input_topic_, 10,
                std::bind(&PerceptionNode::image_callback, this, std::placeholders::_1));
            RCLCPP_INFO(this->get_logger(), "Mode: Camera stream");
        }
    }

private:
    int frame_count_ = 0;
    // ================= Camera Intrinsics =================
    struct CameraIntrinsics
    {
        double fx, fy, cx, cy;
    } intr_;

    // ================= TF2 =================
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    // ================= Image and INFO Callback =================
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        if (!camera_info_received_)
        {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                 "Waiting for camera_info...");
            return;
        }

        cv::Mat frame;
        try
        {
            frame = cv_bridge::toCvShare(msg, "bgr8")->image;
        }
        catch (const cv_bridge::Exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge error: %s", e.what());
            return;
        }
        process_and_publish(frame, msg->header);
    }

    void camera_info_callback(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
    {
        if (camera_info_received_)
            return; // Only read once

        intr_.fx = msg->k[0]; // fx
        intr_.fy = msg->k[4]; // fy
        intr_.cx = msg->k[2]; // cx
        intr_.cy = msg->k[5]; // cy

        camera_info_received_ = true;

        RCLCPP_INFO(this->get_logger(),
                    "Camera intrinsics received: fx=%.2f, fy=%.2f, cx=%.2f, cy=%.2f",
                    intr_.fx, intr_.fy, intr_.cx, intr_.cy);
    }

    void depth_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        try
        {
            if (msg->encoding == "32FC1")
            {
                depth_map_ = cv_bridge::toCvShare(msg, "32FC1")->image.clone();
            }
            else if (msg->encoding == "16UC1")
            {
                cv::Mat tmp = cv_bridge::toCvShare(msg, "16UC1")->image;
                tmp.convertTo(depth_map_, CV_32F, 0.001); // convert mm → meters
            }
            else
            {
                RCLCPP_WARN(this->get_logger(), "Unsupported depth encoding: %s", msg->encoding.c_str());
                return;
            }
            depth_received_ = true;
        }
        catch (const cv_bridge::Exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "Depth cv_bridge error: %s", e.what());
        }
    }

    // ================= Core Processing =================
    void process_and_publish(const cv::Mat &frame,
                             const std_msgs::msg::Header &header)
    {
        cv::Mat vis = frame.clone();
        double u, v;
        double depth_raw = 2.0; // Default fallback depth

        // 1. ALWAYS PROCESS DEPTH (if activated)
        if (activate_depth_ && depth_initialized_)
        {
            rclcpp::Time now_time = this->now();

            if (!depth_valid_ ||
                (now_time - last_depth_time_).seconds() >= (1.0 / depth_fps_))
            {
                cached_depth_map_ = infer_depth_map(frame);
                last_depth_time_ = now_time;
                depth_valid_ = true;
            }

            if (depth_valid_)
            {
                publish_depth_map(cached_depth_map_, header);
            }
        }

        // bool found = extract_target(frame, u, v);
        DetectionResult result = detector_.detect(frame);
        bool found = result.found;
        u = result.u;
        v = result.v;

        if (!found)
        {
            // Publish pixel as -1
            publish_pixel(-1, -1, -1, header);

            // Publish pose as -1
            geometry_msgs::msg::PoseStamped empty_pose;
            empty_pose.header = header;
            empty_pose.header.frame_id = "base_link";
            empty_pose.pose.position.x = -1.0;
            empty_pose.pose.position.y = -1.0;
            empty_pose.pose.position.z = -1.0;
            empty_pose.pose.orientation.w = 1.0;
            pose_publisher_->publish(empty_pose);

            // Publish original image no target
            publish_image(vis, header);
            return;
        }

        cv::Mat depth_map_resized;

        if (activate_depth_ && depth_initialized_ && depth_valid_)
        {
            int px = std::clamp(static_cast<int>(v), 0, frame.rows - 1);
            int py = std::clamp(static_cast<int>(u), 0, frame.cols - 1);
            depth_raw = cached_depth_map_.at<float>(px, py);
            depth_raw = depth_raw / 1000;
            if (depth_max_range_ > 0.0)
            {
                depth_raw = std::min(depth_raw, depth_max_range_); // Clamp to max range
            }
        }

        publish_pixel(u, v, depth_raw, header);
        publish_pose_tf(u, v, depth_raw, header);

        cv::circle(vis, cv::Point(u, v), 8, {0, 255, 0}, 2);
        publish_image(vis, header);
    }

    // ================= Dummy Image =================
    void publish_dummy_image()
    {
        cv::Mat img(480, 640, CV_8UC3, cv::Scalar(40, 40, 40));

        int cx = 320, cy = 240, r = 150;
        int x = cx + static_cast<int>(r * std::cos(dummy_angle_));
        int y = cy + static_cast<int>(r * std::sin(dummy_angle_));

        cv::circle(img, {x, y}, 20, {0, 0, 255}, -1);

        std_msgs::msg::Header header;
        header.stamp = now();
        header.frame_id = "camera_link";

        publish_pixel(x, y, 10.0, header);
        publish_pose_tf(x, y, 10.0, header);

        dummy_angle_ += 0.1;
        publish_image(img, header);
    }

    // ================= Target Extraction =================
    bool extract_target(const cv::Mat &frame, double &u, double &v)
    {
        cv::Mat mask;
        cv::inRange(frame, cv::Scalar(0, 0, 150), cv::Scalar(80, 80, 255), mask);

        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        if (contours.empty())
        {
            u = v = -1.0;
            return false;
        }

        auto largest = std::max_element(
            contours.begin(), contours.end(),
            [](const std::vector<cv::Point> &a, const std::vector<cv::Point> &b)
            {
                return cv::contourArea(a) < cv::contourArea(b);
            });

        cv::Moments m = cv::moments(*largest);
        if (m.m00 <= 0.0)
        {
            u = v = -1.0;
            return false;
        }

        u = m.m10 / m.m00;
        v = m.m01 / m.m00;
        return true;
    }

    // ================= Geometry =================
    geometry_msgs::msg::Point pixelToCamera(double u, double v, double depth)
    {
        geometry_msgs::msg::Point p;

        if (use_external_depth_ && depth_received_)
        {
            // use your external depth map
            int px = std::clamp(static_cast<int>(v), 0, depth_map_.rows - 1);
            int py = std::clamp(static_cast<int>(u), 0, depth_map_.cols - 1);
            depth = depth_map_.at<float>(px, py); // depth in meters
        }

        p.x = (u - intr_.cx) * depth / intr_.fx; // Horizontal offset remains X
        p.y = depth;                             // Depth mapped to Y axis
        p.z = (v - intr_.cy) * depth / intr_.fy; // Vertical offset mapped to Z
        return p;
    }

    // ================= TF-based Pose =================
    void publish_pose_tf(double u, double v, double depth_raw,
                         const std_msgs::msg::Header &header)
    {
        geometry_msgs::msg::PoseStamped pose;
        pose.header = header;
        pose.header.frame_id = "base_link";

        if (use_dummy_target_)
        {
            geometry_msgs::msg::PoseStamped pose;
            pose.header = header;
            pose.header.frame_id = "base_link";
            pose.pose.position.x = u;
            pose.pose.position.y = v;
            pose.pose.position.z = 1.0;
            pose.pose.orientation.w = 1.0;
            pose_publisher_->publish(pose);
            return;
        }

        // Check for invalid inputs
        if (std::isnan(u) || std::isnan(v) || std::isnan(depth_raw))
        {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                 "Invalid target or depth (NaN) detected. Publishing 0,0,0 pose.");
            pose.pose.position.x = 0.0;
            pose.pose.position.y = 0.0;
            pose.pose.position.z = 0.0;
            pose.pose.orientation.w = 1.0;
            pose_publisher_->publish(pose);
            return;
        }

        geometry_msgs::msg::PointStamped cam_pt;
        cam_pt.header = header;
        cam_pt.header.frame_id = "camera_link";
        cam_pt.point = pixelToCamera(u, v, depth_raw);
        cam_pt.point.z *= -1.0; // Adjust for camera coordinate system

        geometry_msgs::msg::PointStamped base_pt;

        try
        {
            // Check if TF is available
            if (tf_buffer_->canTransform("base_link", "camera_link", tf2::TimePointZero))
            {
                base_pt = tf_buffer_->transform(cam_pt, "base_link");
                pose.pose.position = base_pt.point;
                pose.pose.orientation.w = 1.0;
            }
            else
            {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                     "TF camera_link -> base_link not available. Publishing 0,0,0 pose.");
                pose.pose.position.x = 0.0;
                pose.pose.position.y = 0.0;
                pose.pose.position.z = 0.0;
                pose.pose.orientation.w = 1.0;
            }
        }
        catch (const tf2::TransformException &ex)
        {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                 "TF transform failed: %s. Publishing 0,0,0 pose.", ex.what());
            pose.pose.position.x = 0.0;
            pose.pose.position.y = 0.0;
            pose.pose.position.z = 0.0;
            pose.pose.orientation.w = 1.0;
        }

        pose_publisher_->publish(pose);
    }

    float infer_depth_at_pixel(const cv::Mat &frame, double u, double v)
    {
        if (!depth_initialized_)
            return 10.0f;

        cv::Mat blob = cv::dnn::blobFromImage(frame, 1.0 / 255.0, cv::Size(256, 256), cv::Scalar(), true, false);
        depth_net_.setInput(blob);
        cv::Mat depth_map = depth_net_.forward();

        cv::Mat depth_resized;
        cv::resize(depth_map.reshape(1, 256), depth_resized, frame.size());

        int px = std::clamp(static_cast<int>(v), 0, frame.rows - 1);
        int py = std::clamp(static_cast<int>(u), 0, frame.cols - 1);

        return depth_resized.at<float>(px, py) * 5.0f; // MiDaS scale factor
    }

    cv::Mat infer_depth_map(const cv::Mat &frame)
    {
        cv::Mat blob = cv::dnn::blobFromImage(frame, 1.0 / 255.0, cv::Size(256, 256), cv::Scalar(), true, false);
        depth_net_.setInput(blob);

        cv::Mat depth_map = depth_net_.forward();

        // Ensure we extract a 2D matrix from the 4D blob
        // MiDaS output is usually (1, 256, 256) or (1, 1, 256, 256)
        int sizes[] = {256, 256};
        cv::Mat depth_2d(2, sizes, CV_32F, depth_map.ptr<float>());

        cv::Mat depth_resized;
        cv::resize(depth_2d, depth_resized, frame.size());
        // Scale factor for MiDaS relative depth
        depth_resized *= 5.0f;

        return depth_resized;
    }

    // ================= Helpers =================
    void publish_pixel(double u, double v, double d,
                       const std_msgs::msg::Header &header)
    {
        geometry_msgs::msg::PointStamped p;
        p.header = header;
        p.point.x = u;
        p.point.y = v;
        p.point.z = d;
        target_pub_->publish(p);
    }

    void publish_image(const cv::Mat &img,
                       const std_msgs::msg::Header &header)
    {
        publisher_->publish(
            *cv_bridge::CvImage(header, "bgr8", img).toImageMsg());
    }

    void publish_depth_map(const cv::Mat &depth_map, const std_msgs::msg::Header &header)
    {
        if (depth_map.empty())
            return;

        cv::Mat depth_vis;
        double min_val, max_val;
        cv::minMaxLoc(depth_map, &min_val, &max_val);

        // Prevent division by zero if the image is flat
        double diff = max_val - min_val;
        if (diff > 0.0001)
        {
            depth_map.convertTo(depth_vis, CV_8UC1, 255.0 / diff, -min_val * 255.0 / diff);
        }
        else
        {
            depth_vis = cv::Mat::zeros(depth_map.size(), CV_8UC1);
        }

        // Apply a colormap so it's easier to see in Rviz
        cv::Mat colormapped;
        cv::applyColorMap(depth_vis, colormapped, cv::COLORMAP_JET);

        depth_publisher_->publish(*cv_bridge::CvImage(header, "bgr8", colormapped).toImageMsg());
    }

    // ================= Members =================
    std::string input_topic_;
    bool use_dummy_target_;
    bool activate_filter_;
    bool activate_depth_;
    int filter_size_;
    double dummy_angle_ = 0.0;
    double depth_fps_ = 5.0;
    cv::dnn::Net depth_net_;
    bool depth_initialized_ = false;
    rclcpp::Time last_depth_time_;
    cv::Mat cached_depth_map_;
    bool depth_valid_ = false;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
    bool camera_info_received_ = false;
    TargetDetector detector_;
    std::string mode_str_;
    cv::Mat depth_map_;
    std::string external_depth_topic_;
    bool use_external_depth_ = false;
    bool depth_received_ = false;
    double depth_max_range_ = 5.0;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr target_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr depth_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PerceptionNode>());
    rclcpp::shutdown();
    return 0;
}