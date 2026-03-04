#include "rm_auto_aim/detector/detector.hpp"

#include <algorithm>
#include <cmath>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

namespace rm_auto_aim {

ArmorDetector::ArmorDetector(const DetectorParams& params) : params_(params) {}

std::vector<Armor> ArmorDetector::detect(const cv::Mat& input, Color detect_color) {
    // 1. 预处理生成二值图（通道相减，增强目标颜色灯条）
    preprocess(input, detect_color);

    if (params_.debug) {
        debug_img_ = input.clone();
    }

    // 2. 灯条检测
    auto lights = detectLights(input, detect_color);

    // 3. 按x坐标排序
    std::sort(lights.begin(), lights.end(),
              [](const Light& a, const Light& b) { return a.center.x < b.center.x; });

    // 4. 灯条配对生成装甲板
    auto armors = matchArmors(lights);

    if (params_.debug) {
        // 绘制灯条
        for (const auto& light : lights) {
            cv::Point2f pts[4];
            light.points(pts);
            for (int i = 0; i < 4; i++) {
                cv::line(debug_img_, pts[i], pts[(i + 1) % 4],
                         light.color == Color::RED ? cv::Scalar(0, 0, 255)
                                                   : cv::Scalar(255, 0, 0),
                         2);
            }
        }
        // 绘制装甲板
        for (const auto& armor : armors) {
            auto corners = armor.corners();// 获取装甲板4个角点
            for (size_t i = 0; i < corners.size(); i++) {
                cv::line(debug_img_, corners[i], corners[(i + 1) % corners.size()],
                         cv::Scalar(0, 255, 0), 2);
            }
            cv::putText(debug_img_, armor.number, armor.center(),// 绘制装甲板编号
                        cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 255, 255), 2);
        }
    }

    return armors;
}

void ArmorDetector::preprocess(const cv::Mat& input, Color detect_color) {
    // 分离BGR通道
    std::vector<cv::Mat> channels;
    cv::split(input, channels);
    // channels[0]=B, channels[1]=G, channels[2]=R

    cv::Mat diff;
    if (detect_color == Color::RED) {
        // 红色目标：R-B 通道相减，红色灯条区域亮度高
        cv::subtract(channels[2], channels[0], diff);
    } else {
        // 蓝色目标：B-R 通道相减，蓝色灯条区域亮度高
        cv::subtract(channels[0], channels[2], diff);
    }

    // 对通道差值图进行二值化
    cv::threshold(diff, binary_, params_.binary_threshold, 255, cv::THRESH_BINARY);

    // 轻微膨胀以连接断裂的灯条区域
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
    cv::dilate(binary_, binary_, kernel);
}

std::vector<Light> ArmorDetector::detectLights(const cv::Mat& input, Color detect_color) {
    std::vector<Light> lights;// 存储检测到的灯条

    // 查找轮廓
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(binary_, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    for (const auto& contour : contours) {
        // 轮廓点数过少则跳过
        if (contour.size() < 5) continue;

        // 拟合旋转矩形
        auto r_rect = cv::minAreaRect(contour);
        Light light(r_rect);

        // 几何约束检查
        if (!isValidLight(light)) continue;

        // 颜色分类
        light.color = classifyLightColor(input, r_rect);
        if (light.color != detect_color) continue;

        lights.push_back(light);
    }

    return lights;
}

bool ArmorDetector::isValidLight(const Light& light) const {
    // 长宽比约束
    double ratio = light.length / std::max(light.width, 1.0f);
    if (ratio < params_.light_min_ratio || ratio > params_.light_max_ratio) {
        return false;
    }

    // 倾斜角度约束（灯条应该近似竖直）
    float angle = light.tilt_angle;
    // 归一化角度到 [-90, 90]
    if (angle > 90.0f) angle -= 180.0f;
    if (angle < -90.0f) angle += 180.0f;
    if (std::abs(angle) > params_.light_max_angle) {
        return false;
    }

    return true;
}

Color ArmorDetector::classifyLightColor(const cv::Mat& input, const cv::RotatedRect& rect) const {
    // 计算包含膨胀余量的矩形ROI，避免在整帧上分配mask
    cv::Rect bounding = rect.boundingRect();
    int expand = 5;  // 膨胀余量（与dilate kernel对应）
    bounding.x = std::max(0, bounding.x - expand);
    bounding.y = std::max(0, bounding.y - expand);
    bounding.width = std::min(input.cols - bounding.x, bounding.width + 2 * expand);
    bounding.height = std::min(input.rows - bounding.y, bounding.height + 2 * expand);

    // 仅在小ROI区域内创建mask
    cv::Mat mask = cv::Mat::zeros(bounding.size(), CV_8UC1);
    cv::Point2f pts[4];
    rect.points(pts);
    std::vector<cv::Point> roi_pts;
    for (int i = 0; i < 4; i++) {
        roi_pts.emplace_back(
            static_cast<int>(pts[i].x) - bounding.x,
            static_cast<int>(pts[i].y) - bounding.y);
    }
    cv::fillConvexPoly(mask, roi_pts, cv::Scalar(255));

    // 扩展ROI区域以包含灯条周围微弱的颜色光晕
    cv::Mat dilated_mask;
    cv::Mat dilate_kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));
    cv::dilate(mask, dilated_mask, dilate_kernel);

    // 截取原图的对应小区域
    cv::Mat roi_img = input(bounding);

    // 方法一：RGB通道差值（保留作为主判据）
    cv::Scalar mean_val = cv::mean(roi_img, dilated_mask);
    double b_mean = mean_val[0];
    double r_mean = mean_val[2];

    double channel_diff = r_mean - b_mean;
    if (std::abs(channel_diff) > params_.light_color_diff_thresh) {
        return channel_diff > 0 ? Color::RED : Color::BLUE;
    }

    // 方法二：当RGB差值不明显时，仅对小ROI区域做HSV转换
    cv::Mat hsv;
    cv::cvtColor(roi_img, hsv, cv::COLOR_BGR2HSV);
    cv::Scalar hsv_mean = cv::mean(hsv, dilated_mask);
    double hue = hsv_mean[0];  // OpenCV HSV中 H 范围 [0, 180]

    // 红色H在 [0,10] 或 [170,180]，蓝色H在 [100,130]
    if (hue < 30 || hue > 160) {
        return Color::RED;
    } else if (hue > 90 && hue < 140) {
        return Color::BLUE;
    }

    return r_mean > b_mean ? Color::RED : Color::BLUE;
}

std::vector<Armor> ArmorDetector::matchArmors(const std::vector<Light>& lights) {
    std::vector<Armor> armors;

    // 记录每个灯条已被使用的情况，避免同一灯条参与多个装甲板
    std::vector<bool> used(lights.size(), false);

    // 收集所有候选装甲板及其"质量评分"（距离图像中心越近越优先）
    struct ArmorCandidate {
        Armor armor;
        size_t left_idx;
        size_t right_idx;
        double score;  // 越小越优先
    };
    std::vector<ArmorCandidate> candidates;

    // 遍历所有灯条对
    for (size_t i = 0; i < lights.size(); i++) {
        for (size_t j = i + 1; j < lights.size(); j++) {
            const auto& left = lights[i];
            const auto& right = lights[j];

            // 检查是否构成有效装甲板
            if (!isValidArmor(left, right)) continue;

            // 检查两灯条之间是否有其他灯条
            if (containsLight(left, right, lights)) continue;

            Armor armor;
            armor.left_light = left;
            armor.right_light = right;

            // 根据灯条间距判断装甲板大小
            double center_dist = cv::norm(left.center - right.center);
            double avg_length = (left.length + right.length) / 2.0;
            double ratio = center_dist / avg_length;

            if (ratio < params_.armor_max_small_center_distance) {
                armor.type = ArmorType::SMALL;
            } else {
                armor.type = ArmorType::LARGE;
            }

            armor.number = "unknown";  // 待分类器填充

            // 评分：灯条长度比越接近1、灯条间距比越合理，得分越小（越好）
            double length_ratio = left.length < right.length
                                      ? left.length / right.length
                                      : right.length / left.length;
            double score = (1.0 - length_ratio) + std::abs(ratio - 2.0);

            candidates.push_back({armor, i, j, score});
        }
    }

    // 按评分排序，优先保留质量高的装甲板
    std::sort(candidates.begin(), candidates.end(),
              [](const ArmorCandidate& a, const ArmorCandidate& b) {
                  return a.score < b.score;
              });

    // 防止重复选取：每个灯条只参与一个装甲板
    for (const auto& c : candidates) {
        if (used[c.left_idx] || used[c.right_idx]) continue;
        armors.push_back(c.armor);
        used[c.left_idx] = true;
        used[c.right_idx] = true;
    }

    return armors;
}

bool ArmorDetector::isValidArmor(const Light& left, const Light& right) const {
    // 灯条间距/平均灯条长度的比值
    double center_dist = cv::norm(left.center - right.center);
    double avg_length = (left.length + right.length) / 2.0;
    double ratio = center_dist / avg_length;

    // 距离约束
    bool small_valid = (ratio >= params_.armor_min_small_center_distance &&
                        ratio <= params_.armor_max_small_center_distance);
    bool large_valid = (ratio >= params_.armor_min_large_center_distance &&
                        ratio <= params_.armor_max_large_center_distance);
    if (!small_valid && !large_valid) {
        return false;
    }

    // 灯条连线角度约束（装甲板应水平）
    double dx = right.center.x - left.center.x;
    double dy = right.center.y - left.center.y;
    double angle = std::abs(std::atan2(dy, dx)) * 180.0 / CV_PI;
    if (angle > params_.armor_max_angle) {
        return false;
    }

    // 灯条长度比约束（两灯条长度不应差距过大）
    double length_ratio = left.length < right.length ? left.length / right.length
                                                     : right.length / left.length;
    if (length_ratio < params_.armor_min_length_ratio) {
        return false;
    }

    // 灯条宽度比约束（两灯条粗细不应差距过大，排除不同距离的灯条误配对）
    double width_ratio = left.width < right.width ? left.width / right.width
                                                  : right.width / left.width;
    if (width_ratio < params_.armor_min_width_ratio) {
        return false;
    }

    // 灯条倾斜角差约束（同一装甲板的两灯条应近似平行）
    float left_angle = left.tilt_angle;
    float right_angle = right.tilt_angle;
    if (left_angle > 90.0f) left_angle -= 180.0f;
    if (left_angle < -90.0f) left_angle += 180.0f;
    if (right_angle > 90.0f) right_angle -= 180.0f;
    if (right_angle < -90.0f) right_angle += 180.0f;
    double angle_diff = std::abs(left_angle - right_angle);
    if (angle_diff > params_.armor_max_light_angle_diff) {
        return false;
    }

    return true;
}

bool ArmorDetector::containsLight(
    const Light& left, const Light& right,
    const std::vector<Light>& lights) const {
    // 检查left和right之间是否包含其他灯条
    float lx = left.center.x;
    float rx = right.center.x;

    for (const auto& light : lights) {
        if (&light == &left || &light == &right) continue;
        if (light.center.x > lx && light.center.x < rx) {
            return true;
        }
    }
    return false;
}

}  // namespace rm_auto_aim

//输入图像 → 颜色增强+二值化+膨胀 → 轮廓检测 → 灯条几何约束筛选 → 灯条颜色匹配 → 灯条排序 → 
//灯条对几何约束筛选 → 中间无灯条检查 → 装甲板类型判定 → 质量评分排序 → 贪心筛选最优装甲板 → 
//绘制调试图 → 输出装甲板列表