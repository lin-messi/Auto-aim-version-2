#include "rm_auto_aim/solver/armor_solver_node.hpp"

#include <cmath>

namespace rm_auto_aim {

ArmorSolverNode::ArmorSolverNode(const rclcpp::NodeOptions& options)
    : Node("armor_solver", options)
{
    RCLCPP_INFO(get_logger(), "ArmorSolverNode 初始化中...");

    declareParameters();
    loadParams();

    // 初始化跟踪器并传入ROS参数
    tracker_ = std::make_unique<ArmorTracker>();
    tracker_->setParams(
        this->get_parameter("tracker.max_match_distance").as_double(),
        this->get_parameter("tracker.max_match_yaw_diff").as_double(),
        this->get_parameter("tracker.tracking_thres").as_int(),
        this->get_parameter("tracker.lost_time_thres").as_double());
    tracker_->setEKFParams(
        this->get_parameter("ekf.sigma2_q_x").as_double(),
        this->get_parameter("ekf.sigma2_q_y").as_double(),
        this->get_parameter("ekf.sigma2_q_z").as_double(),
        this->get_parameter("ekf.sigma2_q_yaw").as_double(),
        this->get_parameter("ekf.sigma2_q_r").as_double(),
        this->get_parameter("ekf.r_x").as_double(),
        this->get_parameter("ekf.r_y").as_double(),
        this->get_parameter("ekf.r_z").as_double(),
        this->get_parameter("ekf.r_yaw").as_double());

    // 订阅装甲板检测结果
    armors_sub_ = this->create_subscription<rm_interfaces::msg::Armors>(
        "/detector/armors", rclcpp::SensorDataQoS(),
        std::bind(&ArmorSolverNode::armorsCallback, this, std::placeholders::_1));

    // 订阅串口反馈数据（弹速、当前云台角度、颜色）
    serial_sub_ = this->create_subscription<rm_interfaces::msg::SerialReceiveData>(
        "/serial/receive", rclcpp::SensorDataQoS(),
        std::bind(&ArmorSolverNode::serialCallback, this, std::placeholders::_1));

    // 发布目标信息
    target_pub_ = this->create_publisher<rm_interfaces::msg::Target>(
        "/solver/target", rclcpp::SensorDataQoS());

    // 发布云台控制命令
    gimbal_cmd_pub_ = this->create_publisher<rm_interfaces::msg::GimbalCmd>(
        "/solver/gimbal_cmd", rclcpp::SensorDataQoS());

    RCLCPP_INFO(get_logger(), "ArmorSolverNode 初始化完成");
}

void ArmorSolverNode::declareParameters() {
    // EKF过程噪声
    this->declare_parameter("ekf.sigma2_q_x", 0.008);
    this->declare_parameter("ekf.sigma2_q_y", 0.008);
    this->declare_parameter("ekf.sigma2_q_z", 0.008);
    this->declare_parameter("ekf.sigma2_q_yaw", 1.30);//偏航角过程噪声方差
    this->declare_parameter("ekf.sigma2_q_r", 98.0);//目标半径过程噪声方差
    // EKF观测噪声
    this->declare_parameter("ekf.r_x", 0.0005);
    this->declare_parameter("ekf.r_y", 0.0005);
    this->declare_parameter("ekf.r_z", 0.0005);
    this->declare_parameter("ekf.r_yaw", 0.005);//偏航角观测噪声方差
    // 跟踪器参数
    this->declare_parameter("tracker.max_match_distance", 0.5);// EKF状态空间距离阈值
    this->declare_parameter("tracker.max_match_yaw_diff", 0.67);// EKF偏航角差阈值(rad, ~38°)
    this->declare_parameter("tracker.tracking_thres", 3);// 连续跟踪帧数阈值
    this->declare_parameter("tracker.lost_time_thres", 3.05);// 临时丢失时间阈值(秒)
    // 弹道参数
    this->declare_parameter("solver.bullet_speed", 30.0);// 初始弹速(m/s)
    this->declare_parameter("solver.gravity", 9.82);// 重力加速度(m/s²)
    this->declare_parameter("solver.resistance", 0.092);// 空气阻力系数
    // 反陀螺参数
    this->declare_parameter("solver.max_tracking_v_yaw", 60.0);// 小陀螺角速度阈值(°/s)
    this->declare_parameter("solver.side_angle", 15.0);// 切换侧边装甲板角度阈值(°)
    this->declare_parameter("solver.coming_angle", 1.222);// 小陀螺出现角(70°)
    this->declare_parameter("solver.leaving_angle", 0.524);// 装甲板远离相机的最大可见角
    // 延迟补偿
    this->declare_parameter("solver.latency_compensation", 0.02);// 系统总延迟(秒): 图像采集+处理+通信+机械响应
    // 开火判据
    this->declare_parameter("solver.fire_yaw_threshold", 0.05);// yaw 允许误差(rad, ~2.9°)
    this->declare_parameter("solver.fire_pitch_threshold", 0.05);// pitch 允许误差(rad)
    // 调试
    this->declare_parameter("debug", false);
}

void ArmorSolverNode::loadParams() {
    // 弹道参数
    bullet_speed_ = this->get_parameter("solver.bullet_speed").as_double();
    gravity_ = this->get_parameter("solver.gravity").as_double();
    resistance_ = this->get_parameter("solver.resistance").as_double();
    trajectory_compensator_.setParams(bullet_speed_, gravity_, resistance_);

    // 反陀螺
    max_tracking_v_yaw_ = this->get_parameter("solver.max_tracking_v_yaw").as_double();
    side_angle_ = this->get_parameter("solver.side_angle").as_double();
    coming_angle_ = this->get_parameter("solver.coming_angle").as_double();
    leaving_angle_ = this->get_parameter("solver.leaving_angle").as_double();

    // 延迟补偿
    latency_compensation_ = this->get_parameter("solver.latency_compensation").as_double();

    // 开火判据
    fire_yaw_threshold_ = this->get_parameter("solver.fire_yaw_threshold").as_double();
    fire_pitch_threshold_ = this->get_parameter("solver.fire_pitch_threshold").as_double();

    debug_ = this->get_parameter("debug").as_bool();
}

void ArmorSolverNode::serialCallback(
    const rm_interfaces::msg::SerialReceiveData::ConstSharedPtr& msg)
{
    // 动态更新弹速（仅当反馈弹速合理时更新）
    if (msg->bullet_speed > 5.0 && msg->bullet_speed < 50.0) {
        bullet_speed_ = msg->bullet_speed;
        trajectory_compensator_.setParams(bullet_speed_, gravity_, resistance_);
    }

    // 更新当前云台角度（用于开火判据）
    cur_yaw_ = msg->cur_yaw;
    cur_pitch_ = msg->cur_pitch;
}

void ArmorSolverNode::armorsCallback(
    const rm_interfaces::msg::Armors::ConstSharedPtr& msg)
{
    // 计算dt
    rclcpp::Time now = msg->header.stamp;
    double dt = 0.01;  // 默认10ms
    if (!first_frame_) {
        dt = (now - last_time_).seconds();
        if (dt <= 0 || dt > 1.0) dt = 0.01;
    }
    first_frame_ = false;
    last_time_ = now;

    // 更新跟踪器（含EKF预测+更新）
    tracker_->update(*msg, dt);

    // 构造Target消息
    rm_interfaces::msg::Target target_msg;
    target_msg.header = msg->header;

    auto tracker_state = tracker_->state();
    if (tracker_state == TrackerState::TRACKING ||
        tracker_state == TrackerState::TEMP_LOST) {
        target_msg.tracking = true;
        target_msg.id = tracker_->trackedId();
        target_msg.armors_num = tracker_->targetArmorsNum();

        auto state = tracker_->getState();
        // EKF状态: [xc, v_xc, yc, v_yc, zc, v_zc, yaw, v_yaw, r, d_zc]
        target_msg.position.x = state(0);
        target_msg.position.y = state(2);
        target_msg.position.z = state(4);
        target_msg.velocity.x = state(1);
        target_msg.velocity.y = state(3);
        target_msg.velocity.z = state(5);
        target_msg.yaw = state(6);
        target_msg.v_yaw = state(7);
        target_msg.radius_1 = state(8);
        target_msg.d_zc = state(9);

        // --- 射击延迟补偿 ---
        // 将EKF状态向前预测 latency_compensation_ 秒
        // 补偿从"看到目标"到"子弹到达"的整体延迟
        Eigen::VectorXd predicted_state = state;
        double total_latency = latency_compensation_;

        // 先估算弹丸飞行时间，加入总延迟
        double v_yaw = state(7);
        Eigen::Vector3d raw_aim = calcAimPoint(state, v_yaw);
        double horizontal_dist = std::sqrt(
            raw_aim.x() * raw_aim.x() + raw_aim.z() * raw_aim.z());
        // 用粗略的pitch估计飞行时间（而非pitch=0，否则水平速度分量偏大导致飞行时间偏小）
        double rough_pitch = std::atan2(raw_aim.y(), horizontal_dist);
        double fly_time = trajectory_compensator_.calcFlyTime(horizontal_dist, rough_pitch);
        if (fly_time > 0) {
            total_latency += fly_time;
        }

        // 用匀速模型向前预测
        predicted_state(0) += predicted_state(1) * total_latency;   // xc
        predicted_state(2) += predicted_state(3) * total_latency;   // yc
        predicted_state(4) += predicted_state(5) * total_latency;   // zc
        predicted_state(6) += predicted_state(7) * total_latency;   // yaw

        // 用补偿后的状态计算瞄准点
        Eigen::Vector3d aim_point = calcAimPoint(predicted_state, v_yaw);

        // 弹道补偿：计算pitch
        double compensated_pitch = trajectory_compensator_.compensate(
            aim_point.x(), aim_point.y(), aim_point.z());

        // 计算yaw
        double yaw_cmd = std::atan2(aim_point.x(), aim_point.z());

        // 手动补偿
        double dist = aim_point.norm();
        auto manual_comp = manual_compensator_.getCompensation(dist);
        compensated_pitch += manual_comp.pitch_offset;
        yaw_cmd += manual_comp.yaw_offset;

        // --- 开火判据 ---
        double yaw_err = std::abs(yaw_cmd - cur_yaw_);
        while (yaw_err > M_PI) yaw_err = std::abs(yaw_err - 2 * M_PI);
        double pitch_err = std::abs(compensated_pitch - cur_pitch_);

        bool fire = (tracker_state == TrackerState::TRACKING) &&
                    shouldFire(yaw_err, pitch_err);

        // 发布云台控制命令
        rm_interfaces::msg::GimbalCmd gimbal_cmd;
        gimbal_cmd.header = msg->header;
        gimbal_cmd.yaw = yaw_cmd;
        gimbal_cmd.pitch = compensated_pitch;
        gimbal_cmd.fire = fire;
        gimbal_cmd_pub_->publish(gimbal_cmd);

    } else {
        target_msg.tracking = false;
    }

    target_pub_->publish(target_msg);
}

Eigen::Vector3d ArmorSolverNode::calcAimPoint(
    const Eigen::VectorXd& state, double v_yaw)
{
    if (isSmallGyro(v_yaw)) {
        // 小陀螺模式：选择最优装甲板
        return selectBestArmor(state, tracker_->targetArmorsNum());
    }

    // 普通模式：直接瞄准预测的装甲板位置
    double xc = state(0), yc = state(2), zc = state(4);
    double yaw = state(6), r = state(8), d_zc = state(9);

    Eigen::Vector3d aim;
    aim.x() = xc - r * std::cos(yaw);
    aim.y() = yc - r * std::sin(yaw);
    aim.z() = zc + d_zc;

    return aim;
}

bool ArmorSolverNode::isSmallGyro(double v_yaw) const {
    return std::abs(v_yaw) > max_tracking_v_yaw_;
}

bool ArmorSolverNode::shouldFire(double yaw_err, double pitch_err) const {
    return yaw_err < fire_yaw_threshold_ && pitch_err < fire_pitch_threshold_;
}

Eigen::Vector3d ArmorSolverNode::selectBestArmor(
    const Eigen::VectorXd& state, int armors_num)
{
    double xc = state(0), yc = state(2), zc = state(4);
    double yaw = state(6), r = state(8), d_zc = state(9);

    // 计算所有装甲板位置
    double angle_step = 2.0 * M_PI / armors_num;
    double min_yaw_diff = std::numeric_limits<double>::max();
    Eigen::Vector3d best_point;
    bool found_valid = false;

    // coming_angle_: 装甲板正对相机时，最大允许偏角（超过此角度视为不可见）
    double max_visible_angle = coming_angle_;

    for (int i = 0; i < armors_num; i++) {
        double armor_yaw = yaw + i * angle_step;

        // 归一化到[-pi, pi]
        while (armor_yaw > M_PI) armor_yaw -= 2 * M_PI;
        while (armor_yaw < -M_PI) armor_yaw += 2 * M_PI;

        Eigen::Vector3d armor_pos;
        armor_pos.x() = xc - r * std::cos(armor_yaw);
        armor_pos.y() = yc - r * std::sin(armor_yaw);
        armor_pos.z() = zc + ((i % 2 == 0) ? d_zc : -d_zc);

        // 计算装甲板朝向与相机方向的夹角
        double yaw_to_cam = std::atan2(armor_pos.x(), armor_pos.z());
        double yaw_diff = std::abs(yaw_to_cam);

        // 只考虑在可见角度范围内的装甲板
        if (yaw_diff > max_visible_angle) continue;

        if (yaw_diff < min_yaw_diff) {
            min_yaw_diff = yaw_diff;
            best_point = armor_pos;
            found_valid = true;
        }
    }

    // 如果没有可见装甲板（全部背对），回退到最正对的那块
    if (!found_valid) {
        for (int i = 0; i < armors_num; i++) {
            double armor_yaw = yaw + i * angle_step;
            while (armor_yaw > M_PI) armor_yaw -= 2 * M_PI;
            while (armor_yaw < -M_PI) armor_yaw += 2 * M_PI;

            Eigen::Vector3d armor_pos;
            armor_pos.x() = xc - r * std::cos(armor_yaw);
            armor_pos.y() = yc - r * std::sin(armor_yaw);
            armor_pos.z() = zc + ((i % 2 == 0) ? d_zc : -d_zc);

            double yaw_to_cam = std::atan2(armor_pos.x(), armor_pos.z());
            double yaw_diff = std::abs(yaw_to_cam);

            if (yaw_diff < min_yaw_diff) {
                min_yaw_diff = yaw_diff;
                best_point = armor_pos;
            }
        }
    }

    return best_point;
}

}  // namespace rm_auto_aim

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(rm_auto_aim::ArmorSolverNode)
