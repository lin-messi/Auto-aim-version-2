#include "rm_hardware_driver/camera_driver_node.hpp"

#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc.hpp>

namespace rm_auto_aim {

CameraDriverNode::CameraDriverNode(const rclcpp::NodeOptions& options)
    : Node("camera_driver", options)
{
    RCLCPP_INFO(get_logger(), "CameraDriverNode 初始化中...");

    // 声明参数
    this->declare_parameter("camera_id", 0);
    this->declare_parameter("video_path", "");
    this->declare_parameter("frame_width", 640);
    this->declare_parameter("frame_height", 480);
    this->declare_parameter("fps", 30);

    // 相机内参参数
    this->declare_parameter("camera_matrix",
        std::vector<double>{640.0, 0.0, 320.0, 0.0, 640.0, 240.0, 0.0, 0.0, 1.0});
    this->declare_parameter("distortion_model", "plumb_bob");
    this->declare_parameter("distortion_coefficients",
        std::vector<double>{0.0, 0.0, 0.0, 0.0, 0.0});

    camera_id_ = this->get_parameter("camera_id").as_int();
    video_path_ = this->get_parameter("video_path").as_string();
    frame_width_ = this->get_parameter("frame_width").as_int();
    frame_height_ = this->get_parameter("frame_height").as_int();
    fps_ = this->get_parameter("fps").as_int();

    // 加载内参
    loadCameraInfo();

    // 创建发布器
    camera_pub_ = image_transport::create_camera_publisher(this, "/image_raw");

    // 打开相机
    bool opened = false;
    if (!video_path_.empty()) {
        opened = cap_.open(video_path_);
        RCLCPP_INFO(get_logger(), "使用视频文件: %s", video_path_.c_str());
    } else {
        cap_.open(camera_id_, cv::CAP_V4L2);
        if (cap_.isOpened()) {
            cap_.set(cv::CAP_PROP_FRAME_WIDTH, frame_width_);
            cap_.set(cv::CAP_PROP_FRAME_HEIGHT, frame_height_);
            cap_.set(cv::CAP_PROP_FPS, fps_);
            // 设置MJPG格式以提高帧率
            cap_.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));
            opened = true;
        }
        RCLCPP_INFO(get_logger(), "使用相机ID: %d", camera_id_);
    }

    if (!opened || !cap_.isOpened()) {
        RCLCPP_ERROR(get_logger(), "无法打开相机/视频源！");
        return;
    }

    RCLCPP_INFO(get_logger(), "相机已打开: %dx%d @ %d fps",
                frame_width_, frame_height_, fps_);

    // 启动采集线程
    running_ = true;
    capture_thread_ = std::thread(&CameraDriverNode::captureLoop, this);

    RCLCPP_INFO(get_logger(), "CameraDriverNode 初始化完成");
}

CameraDriverNode::~CameraDriverNode() {
    running_ = false;
    if (capture_thread_.joinable()) {
        capture_thread_.join();
    }
    if (cap_.isOpened()) {
        cap_.release();
    }
}

void CameraDriverNode::captureLoop() {
    cv::Mat frame;
    auto target_duration = std::chrono::microseconds(1000000 / fps_);

    while (running_ && rclcpp::ok()) {
        auto start = std::chrono::steady_clock::now();

        if (!cap_.read(frame) || frame.empty()) {
            // 视频文件播完可循环
            if (!video_path_.empty()) {
                cap_.set(cv::CAP_PROP_POS_FRAMES, 0);
                continue;
            }
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "相机读取失败");
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            continue;
        }

        // 转换为ROS消息
        auto stamp = this->now();

        auto img_msg = cv_bridge::CvImage(
            std_msgs::msg::Header(), "bgr8", frame).toImageMsg();
        img_msg->header.stamp = stamp;
        img_msg->header.frame_id = "camera_optical_frame";

        camera_info_msg_.header.stamp = stamp;
        camera_info_msg_.header.frame_id = "camera_optical_frame";

        // 发布
        camera_pub_.publish(*img_msg, camera_info_msg_);

        // 帧率控制
        auto elapsed = std::chrono::steady_clock::now() - start;
        if (elapsed < target_duration) {
            std::this_thread::sleep_for(target_duration - elapsed);
        }
    }
}

void CameraDriverNode::loadCameraInfo() {
    camera_matrix_ = this->get_parameter("camera_matrix").as_double_array();
    dist_coeffs_ = this->get_parameter("distortion_coefficients").as_double_array();
    distortion_model_ = this->get_parameter("distortion_model").as_string();

    // 填充CameraInfo消息
    camera_info_msg_.width = frame_width_;
    camera_info_msg_.height = frame_height_;
    camera_info_msg_.distortion_model = distortion_model_;

    // K矩阵 (3x3) -> 展平为9元素数组
    if (camera_matrix_.size() == 9) {
        for (size_t i = 0; i < 9; i++) {
            camera_info_msg_.k[i] = camera_matrix_[i];
        }
    }

    // 畸变系数
    camera_info_msg_.d = dist_coeffs_;

    // P矩阵 (3x4投影矩阵，简单起见用K填充)
    camera_info_msg_.p[0] = camera_info_msg_.k[0];
    camera_info_msg_.p[1] = camera_info_msg_.k[1];
    camera_info_msg_.p[2] = camera_info_msg_.k[2];
    camera_info_msg_.p[3] = 0.0;
    camera_info_msg_.p[4] = camera_info_msg_.k[3];
    camera_info_msg_.p[5] = camera_info_msg_.k[4];
    camera_info_msg_.p[6] = camera_info_msg_.k[5];
    camera_info_msg_.p[7] = 0.0;
    camera_info_msg_.p[8] = camera_info_msg_.k[6];
    camera_info_msg_.p[9] = camera_info_msg_.k[7];
    camera_info_msg_.p[10] = camera_info_msg_.k[8];
    camera_info_msg_.p[11] = 0.0;

    // R矩阵 (单位矩阵)
    camera_info_msg_.r[0] = 1.0;
    camera_info_msg_.r[4] = 1.0;
    camera_info_msg_.r[8] = 1.0;
}

}  // namespace rm_auto_aim

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(rm_auto_aim::CameraDriverNode)
