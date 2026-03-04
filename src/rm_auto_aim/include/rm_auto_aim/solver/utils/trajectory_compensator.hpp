#pragma once

#include <cmath>

namespace rm_auto_aim {

/**
 * @brief 弹道补偿器
 *
 * 基于抛物线弹道模型（含空气阻力），迭代求解发射仰角。
 * 输入目标三维坐标，输出补偿后的 pitch 角。
 */
class TrajectoryCompensator {
public:
    TrajectoryCompensator() = default;

    /**
     * @brief 设置弹道参数
     * @param bullet_speed 弹丸初速 (m/s)
     * @param gravity 重力加速度 (m/s²)
     * @param resistance 空气阻力系数
     */
    void setParams(double bullet_speed, double gravity, double resistance) {
        bullet_speed_ = bullet_speed;
        gravity_ = gravity;
        resistance_ = resistance;
    }

    /**
     * @brief 补偿弹道下坠，计算需要的pitch角
     * @param x 目标水平距离 (m)
     * @param y 目标竖直高度 (m，向上为正)
     * @param z 目标纵深距离 (m)
     * @return 补偿后的pitch角 (rad)
     */
    double compensate(double x, double y, double z) const {
        // 水平距离
        double horizontal_dist = std::sqrt(x * x + z * z);

        // 无补偿的直线pitch
        double pitch = std::atan2(y, horizontal_dist);

        // 迭代求解（牛顿法思想）
        for (int i = 0; i < max_iterations_; i++) {
            // 用当前pitch估计飞行时间
            double fly_time = calcFlyTime(horizontal_dist, pitch);
            if (fly_time <= 0) break;

            // 计算在该时间内的弹道下坠量
            double drop = calcBulletDrop(fly_time);

            // 实际需要补偿的高度 = 目标高度 + 下坠量
            double target_y = y + drop;

            // 更新pitch
            double new_pitch = std::atan2(target_y, horizontal_dist);

            // 收敛判断
            if (std::abs(new_pitch - pitch) < 1e-6) {
                return new_pitch;
            }
            pitch = new_pitch;
        }

        return pitch;
    }

    /**
     * @brief 计算飞行时间
     * @param dist 水平距离 (m)
     * @param pitch 发射仰角 (rad)
     * @return 飞行时间 (s)
     */
    double calcFlyTime(double dist, double pitch) const {
        double v_horizontal = bullet_speed_ * std::cos(pitch);
        if (v_horizontal < 1e-3) return -1;

        if (resistance_ < 1e-6) {
            // 无阻力模型
            return dist / v_horizontal;
        }

        // 含阻力模型: v(t) = v0 * exp(-k*t)
        // x(t) = v0/k * (1 - exp(-k*t))
        // 数值迭代求解
        double t = dist / v_horizontal;  // 初始估计
        for (int i = 0; i < 20; i++) {
            double x_t = v_horizontal / resistance_ * (1 - std::exp(-resistance_ * t));
            double dx = dist - x_t;
            double v_t = v_horizontal * std::exp(-resistance_ * t);
            if (v_t < 1e-3) break;
            t += dx / v_t;
            if (std::abs(dx) < 1e-4) break;
        }
        return t;
    }

    /**
     * @brief 计算弹道下坠量
     * @param t 飞行时间 (s)
     * @return 下坠量 (m, 正值表示向下)
     */
    double calcBulletDrop(double t) const {
        return 0.5 * gravity_ * t * t;
    }

private:
    double bullet_speed_ = 30.0;    // m/s
    double gravity_ = 9.82;         // m/s²
    double resistance_ = 0.092;     // 空气阻力系数
    int max_iterations_ = 20;       // 迭代次数上限
};

/**
 * @brief 手动补偿表
 *
 * 按距离区间配置 pitch/yaw 偏移量，用于修正系统误差。
 */
struct ManualCompensation {
    double distance_min = 0;
    double distance_max = 0;
    double pitch_offset = 0;    // rad
    double yaw_offset = 0;      // rad
};

class ManualCompensator {
public:
    void addEntry(const ManualCompensation& entry) {
        table_.push_back(entry);
    }

    /**
     * @brief 根据距离查表获取补偿值
     */
    ManualCompensation getCompensation(double distance) const {
        for (const auto& entry : table_) {
            if (distance >= entry.distance_min && distance < entry.distance_max) {
                return entry;
            }
        }
        return {};  // 无匹配则返回零补偿
    }

private:
    std::vector<ManualCompensation> table_;
};

}  // namespace rm_auto_aim
