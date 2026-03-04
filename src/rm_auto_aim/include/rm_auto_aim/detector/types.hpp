#pragma once

#include <opencv2/core.hpp>
#include <string>
#include <vector>

namespace rm_auto_aim {

// 装甲板尺寸常量 (mm)
constexpr double SMALL_ARMOR_WIDTH = 133.0;
constexpr double LARGE_ARMOR_WIDTH = 227.0;
constexpr double ARMOR_HEIGHT = 56.0;

// 颜色枚举
enum class Color : uint8_t {
    BLUE = 0,
    RED = 1,
};

// 装甲板类型
enum class ArmorType : uint8_t {
    SMALL = 0,
    LARGE = 1,
};

// 装甲板编号/符号
enum class ArmorSymbol : uint8_t {
    UNKNOWN = 0,
    HERO = 1,       // 1号英雄
    ENGINEER = 2,   // 2号工程
    INFANTRY_3 = 3, // 3号步兵
    INFANTRY_4 = 4, // 4号步兵
    INFANTRY_5 = 5, // 5号步兵
    SENTRY = 6,     // 哨兵
    OUTPOST = 7,    // 前哨站
    BASE = 8,       // 基地
};

// 灯条结构体
struct Light : public cv::RotatedRect {
    Light() = default;
    explicit Light(const cv::RotatedRect& rect)
        : cv::RotatedRect(rect)
    {
        // 确保角度和尺寸的一致性
        if (size.width > size.height) {
            std::swap(size.width, size.height);
            angle += 90.0f;
        }
        // 计算灯条属性
        length = size.height;
        width = size.width;
        tilt_angle = angle;
        // 顶部和底部点
        cv::Point2f pts[4];
        points(pts);
        // 排序点：top 为 y 最小的点
        std::sort(pts, pts + 4, [](const cv::Point2f& a, const cv::Point2f& b) {
            return a.y < b.y;
        });
        top = (pts[0] + pts[1]) / 2;
        bottom = (pts[2] + pts[3]) / 2;
    }

    Color color;
    float length;
    float width;
    float tilt_angle;
    cv::Point2f top;
    cv::Point2f bottom;
};

// 装甲板结构体
struct Armor {
    Light left_light;
    Light right_light;
    ArmorType type;
    ArmorSymbol symbol = ArmorSymbol::UNKNOWN;
    std::string number;
    float confidence = 0.0f;

    // 装甲板中心
    cv::Point2f center() const {
        return (left_light.center + right_light.center) / 2;
    }

    // 装甲板四个角点 (左上, 右上, 右下, 左下)
    std::vector<cv::Point2f> corners() const {
        return {
            left_light.top,
            right_light.top,
            right_light.bottom,
            left_light.bottom
        };
    }

    // 距图像中心的距离
    float distanceToCenter(const cv::Point2f& img_center) const {
        auto c = center();
        return std::sqrt(
            (c.x - img_center.x) * (c.x - img_center.x) +
            (c.y - img_center.y) * (c.y - img_center.y));
    }
};

// 检测参数
struct DetectorParams {
    // 二值化阈值
    int binary_threshold = 90;

    // 灯条参数
    double light_min_ratio = 0.0001;
    double light_max_ratio = 20.0;
    double light_max_angle = 40.0;
    int light_color_diff_thresh = 20;

    // 装甲板匹配参数
    double armor_min_small_center_distance = 0.8;
    double armor_max_small_center_distance = 3.5;
    double armor_min_large_center_distance = 3.5;
    double armor_max_large_center_distance = 8.0;
    double armor_max_angle = 35.0;
    double armor_min_length_ratio = 0.3;         // 左右灯条长度比下限
    double armor_min_width_ratio = 0.3;          // 左右灯条宽度比下限
    double armor_max_light_angle_diff = 15.0;    // 左右灯条倾斜角最大差值(度)

    // 分类器参数
    double classifier_confidence = 0.7;

    // PnP解算参数
    bool optimize_yaw = false;
    double search_range = 140.0;

    // 调试模式
    bool debug = false;
};

}  // namespace rm_auto_aim
