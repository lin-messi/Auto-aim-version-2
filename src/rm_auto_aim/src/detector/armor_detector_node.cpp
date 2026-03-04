#include "rm_auto_aim/detector/armor_detector_node.hpp"

#include <cv_bridge/cv_bridge.h>

#include <geometry_msgs/msg/pose.hpp>
#include <opencv2/calib3d.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

namespace rm_auto_aim {

ArmorDetectorNode::ArmorDetectorNode(const rclcpp::NodeOptions& options)
    : Node("armor_detector", options)
{
    RCLCPP_INFO(get_logger(), "ArmorDetectorNode 初始化中...");

    // 声明ROS参数
    declareParameters();

    // 创建检测器
    auto params = loadParams();
    detector_ = std::make_unique<ArmorDetector>(params);
    debug_ = params.debug;

    // 订阅相机信息（获取内参后创建PnP解算器）
    cam_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
        "/camera_info", rclcpp::SensorDataQoS(),
        std::bind(&ArmorDetectorNode::cameraInfoCallback, this, std::placeholders::_1));

    // 订阅图像
    img_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
        "/image_raw", rclcpp::SensorDataQoS(),
        std::bind(&ArmorDetectorNode::imageCallback, this, std::placeholders::_1));

    // 发布装甲板检测结果
    armors_pub_ = this->create_publisher<rm_interfaces::msg::Armors>(
        "/detector/armors", rclcpp::SensorDataQoS());

    // 订阅串口反馈数据（动态切换目标颜色）
    serial_sub_ = this->create_subscription<rm_interfaces::msg::SerialReceiveData>(
        "/serial/receive", rclcpp::SensorDataQoS(),
        std::bind(&ArmorDetectorNode::serialCallback, this, std::placeholders::_1));

    // 调试发布
    if (debug_) {
        createDebugPublishers();
    }

    RCLCPP_INFO(get_logger(), "ArmorDetectorNode 初始化完成");
}

void ArmorDetectorNode::declareParameters() {
    // 二值化（图像亮度）
    this->declare_parameter("binary_threshold", 90);
    // 灯条参数（长宽比，倾斜度）
    this->declare_parameter("light.min_ratio", 0.0001);
    this->declare_parameter("light.max_ratio", 20.0);
    this->declare_parameter("light.max_angle", 40.0);
    this->declare_parameter("light.color_diff_thresh", 20);
    // 装甲板参数（根据实际装甲板灯条间距修改）
    this->declare_parameter("armor.min_small_center_distance", 0.8);
    this->declare_parameter("armor.max_small_center_distance", 3.5);
    this->declare_parameter("armor.min_large_center_distance", 3.5);
    this->declare_parameter("armor.max_large_center_distance", 8.0);
    this->declare_parameter("armor.max_angle", 35.0);
    this->declare_parameter("armor.min_length_ratio", 0.3);
    this->declare_parameter("armor.min_width_ratio", 0.3);
    this->declare_parameter("armor.max_light_angle_diff", 15.0);
    // 分类器置信度阈值
    this->declare_parameter("classifier.confidence", 0.7);
    // PnP解算参数
    this->declare_parameter("estimator.optimize_yaw", false);
    this->declare_parameter("estimator.search_range", 140.0);
    // 调试
    this->declare_parameter("debug", false);
    // 目标颜色（0=BLUE, 1=RED）
    this->declare_parameter("detect_color", 1);  // 0=BLUE, 1=RED
}

DetectorParams ArmorDetectorNode::loadParams() {
    DetectorParams p;
    p.binary_threshold = this->get_parameter("binary_threshold").as_int();
    p.light_min_ratio = this->get_parameter("light.min_ratio").as_double();
    p.light_max_ratio = this->get_parameter("light.max_ratio").as_double();
    p.light_max_angle = this->get_parameter("light.max_angle").as_double();
    p.light_color_diff_thresh = this->get_parameter("light.color_diff_thresh").as_int();
    p.armor_min_small_center_distance =
        this->get_parameter("armor.min_small_center_distance").as_double();
    p.armor_max_small_center_distance =
        this->get_parameter("armor.max_small_center_distance").as_double();
    p.armor_min_large_center_distance =
        this->get_parameter("armor.min_large_center_distance").as_double();
    p.armor_max_large_center_distance =
        this->get_parameter("armor.max_large_center_distance").as_double();
    p.armor_max_angle = this->get_parameter("armor.max_angle").as_double();
    p.armor_min_length_ratio = this->get_parameter("armor.min_length_ratio").as_double();
    p.armor_min_width_ratio = this->get_parameter("armor.min_width_ratio").as_double();
    p.armor_max_light_angle_diff = this->get_parameter("armor.max_light_angle_diff").as_double();
    p.classifier_confidence = this->get_parameter("classifier.confidence").as_double();
    p.optimize_yaw = this->get_parameter("estimator.optimize_yaw").as_bool();
    p.search_range = this->get_parameter("estimator.search_range").as_double();
    p.debug = this->get_parameter("debug").as_bool();

    detect_color_ = static_cast<Color>(this->get_parameter("detect_color").as_int());
    return p;
}

void ArmorDetectorNode::cameraInfoCallback(
    const sensor_msgs::msg::CameraInfo::ConstSharedPtr& msg) {
    if (cam_info_received_) return;

    // 提取相机内参矩阵 3x3
    cv::Mat camera_matrix(3, 3, CV_64F);
    for (int i = 0; i < 9; i++) {
        camera_matrix.at<double>(i / 3, i % 3) = msg->k[i];
    }

    // 提取畸变系数
    cv::Mat dist_coeffs(1, static_cast<int>(msg->d.size()), CV_64F);
    for (size_t i = 0; i < msg->d.size(); i++) {
        dist_coeffs.at<double>(0, static_cast<int>(i)) = msg->d[i];
    }

    pnp_solver_ = std::make_unique<PnPSolver>(camera_matrix, dist_coeffs);
    cam_info_received_ = true;
    RCLCPP_INFO(get_logger(), "已接收相机内参，PnP解算器已初始化");
}

void ArmorDetectorNode::imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr& msg) {
    // 等待相机内参
    if (!cam_info_received_) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                             "等待相机内参...");
        return;
    }

    // 转换ROS图像到OpenCV
    auto cv_image = cv_bridge::toCvShare(msg, "bgr8");
    const auto& image = cv_image->image;

    // 执行检测
    auto armors = detector_->detect(image, detect_color_);

    // 构造发布消息
    rm_interfaces::msg::Armors armors_msg;
    armors_msg.header = msg->header;

    cv::Point2f img_center(image.cols / 2.0f, image.rows / 2.0f);

    for (auto& armor : armors) {
        cv::Mat rvec, tvec;
        double yaw;

        // PnP解算
        if (!pnp_solver_->solve(armor, rvec, tvec, yaw)) {
            continue;
        }

        // 构造单个装甲板消息
        rm_interfaces::msg::Armor armor_msg;
        armor_msg.number = armor.number;
        armor_msg.type = armor.type == ArmorType::SMALL ? "small" : "large";
        armor_msg.distance_to_image_center = armor.distanceToCenter(img_center);

        // 平移
        armor_msg.pose.position.x = tvec.at<double>(0);
        armor_msg.pose.position.y = tvec.at<double>(1);
        armor_msg.pose.position.z = tvec.at<double>(2);

        // 旋转：rvec → 四元数
        cv::Mat rotation_matrix;
        cv::Rodrigues(rvec, rotation_matrix);
        tf2::Matrix3x3 tf2_rotation(
            rotation_matrix.at<double>(0, 0), rotation_matrix.at<double>(0, 1),
            rotation_matrix.at<double>(0, 2), rotation_matrix.at<double>(1, 0),
            rotation_matrix.at<double>(1, 1), rotation_matrix.at<double>(1, 2),
            rotation_matrix.at<double>(2, 0), rotation_matrix.at<double>(2, 1),
            rotation_matrix.at<double>(2, 2));
        tf2::Quaternion q;
        tf2_rotation.getRotation(q);
        armor_msg.pose.orientation.x = q.x();
        armor_msg.pose.orientation.y = q.y();
        armor_msg.pose.orientation.z = q.z();
        armor_msg.pose.orientation.w = q.w();

        armors_msg.armors.push_back(armor_msg);
    }

    // 发布
    armors_pub_->publish(armors_msg);

    // 调试发布
    if (debug_) {
        publishDebugImages(detector_->getBinaryImage(), detector_->getDebugImage());
        publishMarkers(armors_msg);
    }
}

void ArmorDetectorNode::serialCallback(
    const rm_interfaces::msg::SerialReceiveData::ConstSharedPtr& msg)
{
    // 下位机发来的颜色字段: 0=BLUE(我方蓝色->检测红色), 1=RED(我方红色->检测蓝色)
    // 即收到我方颜色，取反作为检测目标颜色
    Color new_color = (msg->color == 0) ? Color::RED : Color::BLUE;
    if (new_color != detect_color_) {
        detect_color_ = new_color;
        RCLCPP_INFO(get_logger(), "目标颜色已切换为: %s",
                    detect_color_ == Color::RED ? "RED" : "BLUE");
    }
}

void ArmorDetectorNode::createDebugPublishers() {
    binary_pub_ = image_transport::create_publisher(this, "/armor_detector/binary");
    debug_img_pub_ = image_transport::create_publisher(this, "/armor_detector/debug");
    marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
        "/armor_detector/marker", 10);
}

void ArmorDetectorNode::publishDebugImages(const cv::Mat& binary, const cv::Mat& debug_img) {
    if (!binary.empty()) {
        auto binary_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "mono8", binary).toImageMsg();
        binary_pub_.publish(*binary_msg);
    }
    if (!debug_img.empty()) {
        auto debug_msg =
            cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", debug_img).toImageMsg();
        debug_img_pub_.publish(*debug_msg);
    }
}

void ArmorDetectorNode::publishMarkers(const rm_interfaces::msg::Armors& armors_msg) {
    visualization_msgs::msg::MarkerArray marker_array;

    for (size_t i = 0; i < armors_msg.armors.size(); i++) {
        const auto& armor = armors_msg.armors[i];
        visualization_msgs::msg::Marker marker;
        marker.header = armors_msg.header;
        marker.ns = "armors";
        marker.id = static_cast<int>(i);
        marker.type = visualization_msgs::msg::Marker::CUBE;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.pose = armor.pose;
        marker.scale.x = 0.02;//装甲板厚度
        marker.scale.y = armor.type == "small" ? 0.133 : 0.227;//装甲板宽度
        marker.scale.z = 0.056;//装甲板高度
        marker.color.a = 0.8;
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;
        marker.lifetime = rclcpp::Duration::from_seconds(0.1);
        marker_array.markers.push_back(marker);
    }

    marker_pub_->publish(marker_array);
}

}  // namespace rm_auto_aim

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(rm_auto_aim::ArmorDetectorNode)
