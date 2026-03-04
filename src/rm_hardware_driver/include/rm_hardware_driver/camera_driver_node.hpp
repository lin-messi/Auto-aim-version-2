#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <image_transport/image_transport.hpp>
#include <opencv2/videoio.hpp>

#include <thread>
#include <atomic>

namespace rm_auto_aim {

/**
 * @brief 通用相机驱动节点
 *
 * 支持:
 * - USB相机 (V4L2)
 * - 视频文件输入 (用于调试)
 *
 * 工业相机(HIK/Dahua)需另外集成对应SDK。
 * 本节点提供基础的OpenCV VideoCapture驱动。
 */
class CameraDriverNode : public rclcpp::Node {
public:
    explicit CameraDriverNode(const rclcpp::NodeOptions& options);
    ~CameraDriverNode() override;

private:
    /**
     * @brief 相机采集线程
     */
    void captureLoop();

    /**
     * @brief 加载相机内参（从YAML参数）
     */
    void loadCameraInfo();

    // 相机参数
    int camera_id_;
    std::string video_path_;
    int frame_width_;
    int frame_height_;
    int fps_;

    // 相机内参
    std::vector<double> camera_matrix_;
    std::vector<double> dist_coeffs_;
    std::string distortion_model_;

    // OpenCV相机
    cv::VideoCapture cap_;

    // 采集线程
    std::atomic<bool> running_{false};
    std::thread capture_thread_;

    // ROS发布器
    image_transport::CameraPublisher camera_pub_;
    sensor_msgs::msg::CameraInfo camera_info_msg_;
};

}  // namespace rm_auto_aim
