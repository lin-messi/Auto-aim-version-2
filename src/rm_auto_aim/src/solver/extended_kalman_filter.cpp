#include "rm_auto_aim/solver/utils/extended_kalman_filter.hpp"

#include <cmath>

namespace rm_auto_aim {

ExtendedKalmanFilter::ExtendedKalmanFilter(int n_states, int n_obs)
    : n_states_(n_states), n_obs_(n_obs)
{
    x_ = VecX::Zero(n_states);
    P_ = MatXX::Identity(n_states, n_states);
    Q_ = MatXX::Identity(n_states, n_states);
    R_ = MatXX::Identity(n_obs, n_obs);
}

void ExtendedKalmanFilter::setFunctions(
    PredictFunc f, MeasureFunc h,
    PredictJacobianFunc jf, MeasureJacobianFunc jh)
{
    f_ = std::move(f);
    h_ = std::move(h);
    jf_ = std::move(jf);
    jh_ = std::move(jh);
}

void ExtendedKalmanFilter::setNoiseMatrices(const MatXX& Q, const MatXX& R) {
    Q_ = Q;
    R_ = R;
}

void ExtendedKalmanFilter::init(const VecX& x0) {
    x_ = x0;

    // 分状态设置初始协方差：
    // 位置不确定性低（来自PnP测量），速度/角速度/半径不确定性高（未知）
    P_ = MatXX::Zero(n_states_, n_states_);
    P_(0, 0) = 0.01;   // xc: 位置已知
    P_(1, 1) = 1.0;    // v_xc: 速度未知
    P_(2, 2) = 0.01;   // yc
    P_(3, 3) = 1.0;    // v_yc
    P_(4, 4) = 0.01;   // zc
    P_(5, 5) = 1.0;    // v_zc
    P_(6, 6) = 0.05;   // yaw: 从PnP获得，有一定噪声
    P_(7, 7) = 10.0;   // v_yaw: 角速度完全未知
    P_(8, 8) = 0.5;    // r: 旋转半径初始估计不确定
    P_(9, 9) = 0.1;    // d_zc: z偏移量

    initialized_ = true;
}

ExtendedKalmanFilter::VecX ExtendedKalmanFilter::predict(double dt) {
    if (!initialized_) return x_;

    // 使用解析雅可比矩阵 F
    MatXX F = jf_(x_, dt);

    // 状态预测
    x_ = f_(x_, dt);

    // 过程噪声按dt缩放：连续噪声离散化 Qd ≈ Q * dt
    // dt越大不确定性越大，dt越小不确定性越小
    MatXX Q_scaled = Q_ * dt;

    // 协方差预测
    P_ = F * P_ * F.transpose() + Q_scaled;

    return x_;
}

ExtendedKalmanFilter::VecX ExtendedKalmanFilter::update(const Eigen::Vector4d& z) {
    if (!initialized_) return x_;

    // 使用解析观测雅可比 H
    auto H = jh_(x_);

    // 创新（残差）
    Eigen::Vector4d y = z - h_(x_);

    // 将yaw残差归一化到[-pi, pi]
    while (y(3) > M_PI) y(3) -= 2 * M_PI;
    while (y(3) < -M_PI) y(3) += 2 * M_PI;

    // 创新协方差
    Eigen::Matrix4d S = H * P_ * H.transpose() + R_;

    // 使用LLT分解求逆，比直接.inverse()更稳定
    // 若S不正定则回退到直接求逆
    Eigen::Matrix4d S_inv;
    Eigen::LLT<Eigen::Matrix4d> llt(S);
    if (llt.info() == Eigen::Success) {
        S_inv = llt.solve(Eigen::Matrix4d::Identity());
    } else {
        S_inv = S.inverse();
    }

    // 卡尔曼增益
    auto K = P_ * H.transpose() * S_inv;

    // 状态更新
    x_ = x_ + K * y;

    // 更新后将yaw归一化到[-pi, pi]，防止角度漂移
    while (x_(6) > M_PI) x_(6) -= 2 * M_PI;
    while (x_(6) < -M_PI) x_(6) += 2 * M_PI;

    // 协方差更新 (Joseph 形式提高数值稳定性)
    MatXX I = MatXX::Identity(n_states_, n_states_);
    MatXX IKH = I - K * H;
    P_ = IKH * P_ * IKH.transpose() + K * R_ * K.transpose();

    return x_;
}

}  // namespace rm_auto_aim
