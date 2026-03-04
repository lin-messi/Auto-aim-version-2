#pragma once

#include <rclcpp/rclcpp.hpp>

#include "rm_auto_aim/solver/armor_tracker.hpp"
#include "rm_auto_aim/solver/utils/trajectory_compensator.hpp"
#include "rm_interfaces/msg/armors.hpp"
#include "rm_interfaces/msg/gimbal_cmd.hpp"
#include "rm_interfaces/msg/serial_receive_data.hpp"
#include "rm_interfaces/msg/target.hpp"

namespace rm_auto_aim {

/**
 * @brief 装甲板解算ROS2节点
 *
 * 订阅装甲板检测结果，执行：
 * 1. EKF跟踪与预测
 * 2. 瞄准目标选择（反陀螺策略）
 * 3. 弹道补偿（含弹速动态更新）
 * 4. 射击延迟补偿（预测目标在弹丸飞行时间后的位置）
 * 5. 开火判据（瞄准误差小于阈值才开火）
 * 6. 输出云台控制指令
 */
class ArmorSolverNode : public rclcpp::Node {
public:
    explicit ArmorSolverNode(const rclcpp::NodeOptions& options);

private:
    /**
     * @brief 装甲板检测结果回调
     */
    void armorsCallback(const rm_interfaces::msg::Armors::ConstSharedPtr& msg);

    /**
     * @brief 串口反馈数据回调（弹速、当前云台角度等）
     */
    void serialCallback(const rm_interfaces::msg::SerialReceiveData::ConstSharedPtr& msg);

    /**
     * @brief 声明并加载参数
     */
    void declareParameters();
    void loadParams();

    /**
     * @brief 根据EKF状态计算瞄准点
     * @param state EKF状态向量
     * @param v_yaw 目标角速度
     * @return 瞄准点三维坐标
     */
    Eigen::Vector3d calcAimPoint(const Eigen::VectorXd& state, double v_yaw);

    /**
     * @brief 判断是否为小陀螺状态
     */
    bool isSmallGyro(double v_yaw) const;

    /**
     * @brief 选择最优装甲板（反陀螺策略）
     * @param state EKF状态向量
     * @param armors_num 目标装甲板数量
     * @return 选中的装甲板三维坐标
     */
    Eigen::Vector3d selectBestArmor(
        const Eigen::VectorXd& state, int armors_num);

    /**
     * @brief 判断是否满足开火条件
     * @param yaw_err 瞄准yaw误差(rad)
     * @param pitch_err 瞄准pitch误差(rad)
     * @return 是否开火
     */
    bool shouldFire(double yaw_err, double pitch_err) const;

    // 跟踪器
    std::unique_ptr<ArmorTracker> tracker_;

    // 弹道补偿器
    TrajectoryCompensator trajectory_compensator_;
    ManualCompensator manual_compensator_;

    // 订阅与发布
    rclcpp::Subscription<rm_interfaces::msg::Armors>::SharedPtr armors_sub_;
    rclcpp::Subscription<rm_interfaces::msg::SerialReceiveData>::SharedPtr serial_sub_;
    rclcpp::Publisher<rm_interfaces::msg::Target>::SharedPtr target_pub_;
    rclcpp::Publisher<rm_interfaces::msg::GimbalCmd>::SharedPtr gimbal_cmd_pub_;

    // 时间戳管理
    rclcpp::Time last_time_;
    bool first_frame_ = true;

    // 弹道参数
    double bullet_speed_ = 30.0;
    double gravity_ = 9.82;
    double resistance_ = 0.092;

    // 反陀螺参数
    double max_tracking_v_yaw_ = 60.0;   // 角速度阈值
    double side_angle_ = 15.0;            // 切换角度阈值(度)
    double coming_angle_ = 1.222;         // 小陀螺出现角 (70°)
    double leaving_angle_ = 0.524;        // 小陀螺消失角 (30°)

    // 射击延迟补偿参数
    double latency_compensation_ = 0.02;  // 系统总延迟(秒): 图像采集+处理+通信+机械响应

    // 开火判据参数
    double fire_yaw_threshold_ = 0.05;    // yaw 允许误差(rad, ~2.9°)
    double fire_pitch_threshold_ = 0.05;  // pitch 允许误差(rad)

    // 串口反馈状态
    double cur_yaw_ = 0.0;               // 当前云台yaw角(rad)
    double cur_pitch_ = 0.0;             // 当前云台pitch角(rad)

    // 调试
    bool debug_ = false;
};

}  // namespace rm_auto_aim
