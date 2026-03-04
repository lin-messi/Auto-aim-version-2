#pragma once

#include <Eigen/Dense>
#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <vector>

#include "rm_auto_aim/detector/types.hpp"

namespace rm_auto_aim {

/**
 * @brief PnP姿态解算器
 *
 * 使用OpenCV solvePnP计算装甲板在相机坐标系下的三维位姿
 * 输入：装甲板四角图像坐标 + 相机内参
 * 输出：平移向量(x,y,z) + 旋转(yaw)
 */
class PnPSolver {
public:
    PnPSolver(const cv::Mat& camera_matrix, const cv::Mat& dist_coeffs);

    /**
     * @brief 解算装甲板三维位姿
     * @param armor 检测到的装甲板
     * @param rvec [out] 旋转向量
     * @param tvec [out] 平移向量
     * @param yaw [out] 偏航角(弧度)
     * @return 解算是否成功
     */
    bool solve(const Armor& armor, cv::Mat& rvec, cv::Mat& tvec, double& yaw);

    /**
     * @brief 从旋转矩阵提取yaw角
     */
    static double extractYaw(const cv::Mat& rotation_matrix);

    /**
     * @brief 计算装甲板到相机的距离
     */
    static double calculateDistance(const cv::Mat& tvec);

private:
    /**
     * @brief 获取装甲板的三维模型点
     * @param type 装甲板类型（大/小）
     * @return 四个角点的三维坐标 (左上, 右上, 右下, 左下)
     */
    std::vector<cv::Point3f> getObjectPoints(ArmorType type) const;

    cv::Mat camera_matrix_;
    cv::Mat dist_coeffs_;

    // 小装甲板3D点
    std::vector<cv::Point3f> small_armor_points_;
    // 大装甲板3D点
    std::vector<cv::Point3f> large_armor_points_;
};

}  // namespace rm_auto_aim
