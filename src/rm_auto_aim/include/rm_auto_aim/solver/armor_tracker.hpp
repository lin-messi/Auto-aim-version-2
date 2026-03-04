#pragma once

#include <Eigen/Dense>
#include <memory>
#include <string>

#include "rm_auto_aim/solver/utils/extended_kalman_filter.hpp"
#include "rm_interfaces/msg/armor.hpp"
#include "rm_interfaces/msg/armors.hpp"

namespace rm_auto_aim {

/**
 * @brief 跟踪状态枚举
 */
enum class TrackerState : uint8_t {
    LOST = 0,        // 目标丢失
    DETECTING = 1,   // 检测到目标，初始化中
    TRACKING = 2,    // 正在稳定跟踪
    TEMP_LOST = 3,   // 临时丢失（掉帧处理）
};

/**
 * @brief 装甲板跟踪器
 *
 * 状态机:
 *   LOST ←→ DETECTING → TRACKING ←→ TEMP_LOST → LOST
 *
 * 使用 EKF 跟踪目标旋转中心的运动状态。
 * 状态向量: [xc, v_xc, yc, v_yc, zc, v_zc, yaw, v_yaw, r, d_zc]
 */
class ArmorTracker {
public:
    ArmorTracker();

    /**
     * @brief 设置跟踪参数
     */
    void setParams(
        double max_match_distance, double max_match_yaw_diff,
        int tracking_thres, double lost_time_thres);

    /**
     * @brief 设置EKF噪声参数
     */
    void setEKFParams(
        double sigma2_q_x, double sigma2_q_y, double sigma2_q_z,
        double sigma2_q_yaw, double sigma2_q_r,
        double r_x, double r_y, double r_z, double r_yaw);

    /**
     * @brief 更新跟踪器
     * @param armors 当前帧检测结果
     * @param dt 距上一帧的时间间隔
     */
    void update(const rm_interfaces::msg::Armors& armors, double dt);

    /**
     * @brief 获取当前跟踪状态
     */
    TrackerState state() const { return state_; }

    /**
     * @brief 获取跟踪目标ID
     */
    std::string trackedId() const { return tracked_id_; }

    /**
     * @brief 获取EKF状态向量
     */
    Eigen::VectorXd getState() const { return ekf_->state(); }

    /**
     * @brief 获取目标装甲板数量（用于旋转模型）
     */
    int targetArmorsNum() const { return target_armors_num_; }

private:
    /**
     * @brief 初始化EKF
     */
    void initEKF(const rm_interfaces::msg::Armor& armor);

    /**
     * @brief 匹配装甲板（关联检测和跟踪，含ID一致性检查）
     * @param armors 检测到的装甲板
     * @return 匹配到的装甲板索引，-1表示未匹配
     */
    int matchArmor(const rm_interfaces::msg::Armors& armors);

    /**
     * @brief 从Armor消息中提取观测向量 [x, y, z, yaw]
     */
    static Eigen::Vector4d extractMeasurement(const rm_interfaces::msg::Armor& armor);

    /**
     * @brief 在LOST状态下，选择距相机最近的装甲板
     */
    static int selectClosestArmor(const rm_interfaces::msg::Armors& armors);

    /**
     * @brief 根据tracked_id更新目标装甲板数量
     */
    void updateTargetArmorsNum();

    /**
     * @brief 状态转移函数 (运动模型)
     */
    static Eigen::VectorXd predictFunc(const Eigen::VectorXd& x, double dt);

    /**
     * @brief 观测函数 (从旋转中心反推装甲板位置)
     */
    static Eigen::Vector4d measureFunc(const Eigen::VectorXd& x);

    /**
     * @brief 状态转移解析雅可比 (dF/dx)
     */
    static Eigen::MatrixXd predictJacobian(const Eigen::VectorXd& x, double dt);

    /**
     * @brief 观测解析雅可比 (dH/dx)
     */
    static Eigen::Matrix<double, 4, Eigen::Dynamic> measureJacobian(const Eigen::VectorXd& x);

    // EKF
    std::unique_ptr<ExtendedKalmanFilter> ekf_;

    // 跟踪状态
    TrackerState state_ = TrackerState::LOST;
    std::string tracked_id_;
    int target_armors_num_ = 4;   // 默认步兵4块甲

    // 状态计数器
    int detect_count_ = 0;
    int lost_count_ = 0;
    double lost_time_ = 0;

    // 参数
    double max_match_distance_ = 0.5;
    double max_match_yaw_diff_ = 0.67;
    int tracking_thres_ = 3;
    double lost_time_thres_ = 3.05;

    // EKF噪声参数
    double sigma2_q_x_ = 0.008;
    double sigma2_q_y_ = 0.008;
    double sigma2_q_z_ = 0.008;
    double sigma2_q_yaw_ = 1.30;
    double sigma2_q_r_ = 98.0;
    double r_x_ = 0.0005;
    double r_y_ = 0.0005;
    double r_z_ = 0.0005;
    double r_yaw_ = 0.005;
};

}  // namespace rm_auto_aim
