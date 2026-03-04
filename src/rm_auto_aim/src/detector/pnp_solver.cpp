#include "rm_auto_aim/detector/pnp_solver.hpp"

#include <cmath>
#include <opencv2/calib3d.hpp>

namespace rm_auto_aim {

PnPSolver::PnPSolver(const cv::Mat& camera_matrix, const cv::Mat& dist_coeffs)
    : camera_matrix_(camera_matrix.clone()), dist_coeffs_(dist_coeffs.clone())
{
    // 装甲板半宽、半高 (单位: m，原始为mm需除以1000)
    double small_half_w = SMALL_ARMOR_WIDTH / 2.0 / 1000.0;
    double large_half_w = LARGE_ARMOR_WIDTH / 2.0 / 1000.0;
    double half_h = ARMOR_HEIGHT / 2.0 / 1000.0;

    // 模型点坐标系：装甲板中心为原点，x右y下z前
    // 顺序: 左上, 右上, 右下, 左下（与corners()对应）
    small_armor_points_ = {
        cv::Point3f(-small_half_w, -half_h, 0),
        cv::Point3f(small_half_w, -half_h, 0),
        cv::Point3f(small_half_w, half_h, 0),
        cv::Point3f(-small_half_w, half_h, 0),
    };

    large_armor_points_ = {
        cv::Point3f(-large_half_w, -half_h, 0),
        cv::Point3f(large_half_w, -half_h, 0),
        cv::Point3f(large_half_w, half_h, 0),
        cv::Point3f(-large_half_w, half_h, 0),
    };
}

bool PnPSolver::solve(const Armor& armor, cv::Mat& rvec, cv::Mat& tvec, double& yaw) {
    auto image_points = armor.corners();
    auto object_points = getObjectPoints(armor.type);

    // 使用 IPPE_SQUARE 方法，该方法对平面目标（如装甲板）有两个解
    // 需要评估哪个解更合理
    std::vector<cv::Mat> rvecs, tvecs;
    bool success = cv::solvePnPGeneric(
        object_points, image_points,
        camera_matrix_, dist_coeffs_,
        rvecs, tvecs,
        false, cv::SOLVEPNP_IPPE_SQUARE);

    if (!success || rvecs.empty()) {
        return false;
    }

    // 选择最优解：优先选重投影误差更小的解
    double min_error = std::numeric_limits<double>::max();
    int best_idx = 0;

    for (size_t i = 0; i < rvecs.size(); i++) {
        std::vector<cv::Point2f> reproj_points;
        cv::projectPoints(object_points, rvecs[i], tvecs[i],
                          camera_matrix_, dist_coeffs_, reproj_points);

        double error = 0;
        for (size_t j = 0; j < image_points.size(); j++) {
            error += cv::norm(image_points[j] - reproj_points[j]);
        }
        error /= image_points.size();

        if (error < min_error) {
            min_error = error;
            best_idx = static_cast<int>(i);
        }
    }

    rvec = rvecs[best_idx];
    tvec = tvecs[best_idx];

    // 提取yaw角
    cv::Mat rotation_matrix;
    cv::Rodrigues(rvec, rotation_matrix);
    yaw = extractYaw(rotation_matrix);

    return true;
}

double PnPSolver::extractYaw(const cv::Mat& rotation_matrix) {
    // 从旋转矩阵中提取yaw角
    // 使用 atan2(R[2][0], R[0][0]) 提取绕Y轴旋转
    double r00 = rotation_matrix.at<double>(0, 0);
    double r20 = rotation_matrix.at<double>(2, 0);
    return std::atan2(r20, r00);
}

double PnPSolver::calculateDistance(const cv::Mat& tvec) {
    double x = tvec.at<double>(0);
    double y = tvec.at<double>(1);
    double z = tvec.at<double>(2);
    return std::sqrt(x * x + y * y + z * z);
}

std::vector<cv::Point3f> PnPSolver::getObjectPoints(ArmorType type) const {
    return type == ArmorType::SMALL ? small_armor_points_ : large_armor_points_;
}

}  // namespace rm_auto_aim

//初始化：相机参数 + 小/大装甲板3D模型点 → 输入装甲板（角点+类型）→ 
//PnP解算（IPPE_SQUARE）→ 多解筛选（重投影误差最小）→ 
//输出最优rvec/tvec → 旋转矩阵转换 → 提取yaw角 → 
//通过tvec计算相机到装甲板的直线距离