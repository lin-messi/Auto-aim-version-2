#pragma once

#include <image_transport/image_transport.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "rm_auto_aim/detector/detector.hpp"
#include "rm_auto_aim/detector/pnp_solver.hpp"
#include "rm_interfaces/msg/armors.hpp"
#include "rm_interfaces/msg/serial_receive_data.hpp"

namespace rm_auto_aim {

/**
 * @brief 装甲板检测ROS2节点
 *
 * 订阅相机图像，执行灯条检测->装甲板匹配->PnP解算，
 * 发布检测到的装甲板三维位姿信息。
 * 订阅串口反馈数据，动态切换目标颜色。
 */
class ArmorDetectorNode : public rclcpp::Node {
public:
    explicit ArmorDetectorNode(const rclcpp::NodeOptions& options);

private:
    void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr& msg);
    void cameraInfoCallback(const sensor_msgs::msg::CameraInfo::ConstSharedPtr& msg);
    void serialCallback(const rm_interfaces::msg::SerialReceiveData::ConstSharedPtr& msg);

    // 声明和初始化ROS参数
    void declareParameters();
    DetectorParams loadParams();

    // 创建调试发布器
    void createDebugPublishers();
    void publishDebugImages(const cv::Mat& binary, const cv::Mat& debug_img);
    void publishMarkers(const rm_interfaces::msg::Armors& armors_msg);

    // 检测器与PnP解算器
    std::unique_ptr<ArmorDetector> detector_;
    std::unique_ptr<PnPSolver> pnp_solver_;

    // 目标颜色
    Color detect_color_ = Color::RED;

    // 图像订阅
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr img_sub_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr cam_info_sub_;

    // 串口反馈订阅（用于动态切换颜色）
    rclcpp::Subscription<rm_interfaces::msg::SerialReceiveData>::SharedPtr serial_sub_;

    // 检测结果发布
    rclcpp::Publisher<rm_interfaces::msg::Armors>::SharedPtr armors_pub_;

    // 调试发布器
    bool debug_ = false;
    image_transport::Publisher binary_pub_;
    image_transport::Publisher debug_img_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;

    // 相机内参是否已初始化
    bool cam_info_received_ = false;
};

}  // namespace rm_auto_aim
