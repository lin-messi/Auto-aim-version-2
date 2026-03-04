#pragma once

#include <Eigen/Dense>
#include <functional>

namespace rm_auto_aim {

/**
 * @brief 扩展卡尔曼滤波器 (EKF)
 *
 * 状态向量 (10维):
 *   [xc, v_xc, yc, v_yc, zc, v_zc, yaw, v_yaw, r, d_zc]
 * 观测向量 (4维):
 *   [x_armor, y_armor, z_armor, yaw]
 *
 * 运动模型：匀速+匀角速度
 * 观测模型：从旋转中心反推装甲板位置
 */
class ExtendedKalmanFilter {
public:
    using VecX = Eigen::VectorXd;
    using MatXX = Eigen::MatrixXd;

    // 函数签名：状态转移 f(x, dt)、观测模型 h(x)
    // 解析雅可比：Jf(x, dt) 返回 F 矩阵，Jh(x) 返回 H 矩阵
    using PredictFunc = std::function<VecX(const VecX&, double)>;
    using MeasureFunc = std::function<Eigen::Vector4d(const VecX&)>;
    using PredictJacobianFunc = std::function<MatXX(const VecX&, double)>;
    using MeasureJacobianFunc = std::function<Eigen::Matrix<double, 4, Eigen::Dynamic>(const VecX&)>;

    /**
     * @param n_states 状态维度 (10)
     * @param n_obs    观测维度 (4)
     */
    ExtendedKalmanFilter(int n_states, int n_obs);

    /**
     * @brief 设置运动模型和观测模型（含解析雅可比）
     */
    void setFunctions(PredictFunc f, MeasureFunc h,
                      PredictJacobianFunc jf, MeasureJacobianFunc jh);

    /**
     * @brief 设置过程噪声和观测噪声
     */
    void setNoiseMatrices(const MatXX& Q, const MatXX& R);

    /**
     * @brief 初始化状态
     */
    void init(const VecX& x0);

    /**
     * @brief 预测步
     * @param dt 时间间隔(秒)
     * @return 预测后状态
     */
    VecX predict(double dt);

    /**
     * @brief 更新步
     * @param z 观测向量
     * @return 更新后状态
     */
    VecX update(const Eigen::Vector4d& z);

    // 访问器
    VecX state() const { return x_; }
    MatXX covariance() const { return P_; }

private:
    int n_states_;
    int n_obs_;

    VecX x_;      // 状态向量
    MatXX P_;     // 协方差矩阵
    MatXX Q_;     // 过程噪声
    MatXX R_;     // 观测噪声

    PredictFunc f_;   // 状态转移函数
    MeasureFunc h_;   // 观测函数
    PredictJacobianFunc jf_;   // 状态转移解析雅可比
    MeasureJacobianFunc jh_;   // 观测解析雅可比

    bool initialized_ = false;
};

}  // namespace rm_auto_aim
