#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

class DepthNode : public rclcpp::Node
{
public:
    DepthNode() : Node("depth_trt_node")
    {
        sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera/image", 10,
            std::bind(&DepthNode::imageCallback, this, std::placeholders::_1));

        pub_ = this->create_publisher<sensor_msgs::msg::Image>(
            "/perception/depth", 10);

        // TODO: Load TensorRT engine here
        // loadEngine("depth_anything_vits.engine");
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_;

    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        cv::Mat frame = cv_bridge::toCvCopy(msg, "bgr8")->image;

        // ---- Run TensorRT inference ----
        cv::Mat depth = runInference(frame);

        // Normalize for visualization
        cv::Mat depth_vis;
        cv::normalize(depth, depth_vis, 0, 255, cv::NORM_MINMAX);
        depth_vis.convertTo(depth_vis, CV_8U);

        auto out_msg = cv_bridge::CvImage(msg->header, "mono8", depth_vis).toImageMsg();
        pub_->publish(*out_msg);
    }

    cv::Mat runInference(const cv::Mat &frame)
    {
        // TODO: Replace with TensorRT inference
        return cv::Mat(frame.rows, frame.cols, CV_32F, 1.0);
    }
};