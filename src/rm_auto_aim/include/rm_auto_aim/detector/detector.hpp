#pragma once

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <vector>

#include "rm_auto_aim/detector/types.hpp"

namespace rm_auto_aim {

/**
 * @brief 灯条检测与装甲板匹配器
 *
 * 传统CV方案：
 * 1. 图像预处理（灰度+二值化）
 * 2. 轮廓提取与灯条识别
 * 3. 灯条配对匹配装甲板
 */
class ArmorDetector {
public:
    explicit ArmorDetector(const DetectorParams& params);

    /**
     * @brief 主检测流程
     * @param input 输入BGR图像
     * @param detect_color 目标颜色（敌方颜色）
     * @return 检测到的装甲板列表
     */
    std::vector<Armor> detect(const cv::Mat& input, Color detect_color);

    // 获取调试用的二值图
    cv::Mat getBinaryImage() const { return binary_; }
    cv::Mat getDebugImage() const { return debug_img_; }

    // 更新参数
    void setParams(const DetectorParams& params) { params_ = params; }

private:
    /**
     * @brief 图像预处理：生成二值图
     * 使用通道相减增强目标颜色，提升光照鲁棒性
     */
    void preprocess(const cv::Mat& input, Color detect_color);

    /**
     * @brief 检测灯条
     * @param detect_color 目标颜色
     * @return 检测到的灯条列表
     */
    std::vector<Light> detectLights(const cv::Mat& input, Color detect_color);

    /**
     * @brief 灯条是否满足几何约束
     */
    bool isValidLight(const Light& light) const;

    /**
     * @brief 判断灯条颜色
     */
    Color classifyLightColor(const cv::Mat& input, const cv::RotatedRect& rect) const;

    /**
     * @brief 配对灯条，匹配装甲板
     * @param lights 灯条列表（已按x坐标排序）
     * @return 匹配到的装甲板列表
     */
    std::vector<Armor> matchArmors(const std::vector<Light>& lights);

    /**
     * @brief 判断两灯条是否可构成装甲板
     */
    bool isValidArmor(const Light& left, const Light& right) const;

    /**
     * @brief 检查两灯条之间是否包含其他灯条
     */
    bool containsLight(
        const Light& left, const Light& right,
        const std::vector<Light>& lights) const;

    DetectorParams params_;
    cv::Mat binary_;
    cv::Mat debug_img_;
};

}  // namespace rm_auto_aim
