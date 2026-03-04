# RoboMaster Auto-Aim System (v2)

基于传统视觉 + EKF 的 RoboMaster 自瞄系统，使用 ROS2 构建。

## 目录结构

```
auto-aim/src/
├── rm_interfaces/          # 自定义消息与服务
│   ├── msg/                # Armor, Armors, Target, GimbalCmd, ...
│   └── srv/                # SetMode
├── rm_auto_aim/            # 核心算法包
│   ├── detector/           # 识别模块
│   │   ├── types.hpp       # 灯条/装甲板数据结构 + 检测参数
│   │   ├── detector.*      # 灯条检测 + 装甲板匹配
│   │   ├── pnp_solver.*    # PnP三维解算
│   │   └── armor_detector_node.*  # ROS2节点
│   └── solver/             # 解算模块
│       ├── extended_kalman_filter.*  # 10维EKF
│       ├── armor_tracker.*          # 跟踪状态机
│       ├── trajectory_compensator.hpp  # 弹道补偿
│       └── armor_solver_node.*      # ROS2节点
├── rm_hardware_driver/     # 硬件驱动
│   ├── fixed_packet.hpp    # 定长串口协议包
│   ├── serial_driver_node.*  # 串口通信节点
│   └── camera_driver_node.*  # 通用相机驱动
├── rm_bringup/             # 启动配置
│   ├── launch/bringup.launch.py  # 完整启动文件
│   └── config/node_params/       # 各节点参数YAML
└── rm_robot_description/   # 机器人URDF描述
```

## 算法流程

```
相机图像 (BGR)
    ↓
[检测器 Detector]
  通道相减 → 二值化 → 轮廓检测 → 灯条筛选 → 灯条配对 → 装甲板去重
    ↓
[PnP解算器]
  IPPE_SQUARE → 重投影误差筛选 → 3D位姿 (tvec + yaw)
    ↓
[跟踪器 ArmorTracker]
  EKF预测 → 数据关联(位置+yaw+ID) → EKF更新 → 状态机管理
    ↓
[解算节点 ArmorSolverNode]
  延迟补偿(系统延迟+飞行时间) → 反陀螺策略 → 弹道补偿 → 开火判据
    ↓
云台控制指令 (yaw, pitch, fire) → 串口 → MCU
```

## 修改记录

### v2.1 Bug修复与优化

#### 1. [严重] ArmorSolverNode 跟踪器参数未初始化

**问题**: `ArmorSolverNode` 构造函数中创建了 `ArmorTracker`，但从未调用 `setParams()` 和 `setEKFParams()`，导致跟踪器始终使用硬编码默认值，YAML中配置的EKF噪声参数和跟踪阈值完全无效。

**修复**: 在构造函数中创建跟踪器后立即调用 `setParams()` 和 `setEKFParams()`，将ROS参数传入。

**影响文件**: `armor_solver_node.cpp`

#### 2. [严重] DETECTING 状态未执行 EKF predict

**问题**: `ArmorTracker::update()` 中，EKF predict 仅在 TRACKING/TEMP_LOST 状态执行。DETECTING 状态下 EKF 已初始化但不做预测，导致用陈旧的状态做数据关联，匹配容易失败。

**修复**: DETECTING 状态也执行 EKF predict。改为只要不是 LOST 状态就做预测。

**影响文件**: `armor_tracker.cpp`

#### 3. [中等] LOST 状态盲选 armors[0]

**问题**: 进入 LOST 后检测到多个装甲板时，始终选取数组第一个元素初始化。第一个元素可能是最远的或最不可靠的检测。

**修复**: 新增 `selectClosestArmor()` 方法，选择距相机三维距离最近的装甲板初始化。

**影响文件**: `armor_tracker.hpp`, `armor_tracker.cpp`

#### 4. [中等] matchArmor 不检查目标ID一致性

**问题**: 数据关联只看位置距离和yaw差异，不考虑装甲板编号。当两个不同ID的机器人靠近时，跟踪器可能静默切换目标。

**修复**: 对ID不一致的候选装甲板施加距离惩罚（+0.5m），优先匹配同ID目标。

**影响文件**: `armor_tracker.cpp`

#### 5. [中等] 检测器颜色分类每次分配全帧mask

**问题**: `classifyLightColor()` 对每个灯条候选都分配一个全帧大小的 `cv::Mat::zeros(input.size())`，并在RGB差值不明显时对全帧做 HSV 转换。在1280x1024分辨率、数十个灯条候选的情况下，内存分配和转换开销显著。

**修复**: 改为只在灯条 `boundingRect` 的小区域内分配mask和做HSV转换，减少内存分配约100倍。

**影响文件**: `detector.cpp`

#### 6. [中等] 装甲板匹配无去重

**问题**: 同一个灯条可以出现在多个装甲板中（如灯条A-B和灯条A-C都通过验证）。这会导致重复检测，增加PnP解算开销和后续跟踪噪声。

**修复**: 新增基于评分的贪心去重机制。按灯条长度比和间距比评分排序，优先保留质量高的装甲板，每个灯条只参与一个装甲板。

**影响文件**: `detector.cpp`

#### 7. [中等] EKF 过程噪声 Q 不随 dt 缩放

**问题**: 过程噪声矩阵 Q 为常量，不随时间步长 dt 变化。帧率波动时（如30fps跳到15fps），不确定性传播不准确——dt 翻倍但噪声不变，导致协方差偏小。

**修复**: 预测步中使用 `Q_scaled = Q * dt`（连续白噪声离散化的一阶近似）。

**影响文件**: `extended_kalman_filter.cpp`

#### 8. [中等] EKF 初始协方差 P0 = 0.1*I 不合理

**问题**: 所有状态的初始不确定性相同（0.1）。位置来自PnP测量应有较低不确定性，而速度、角速度完全未知应有较高不确定性。不合理的 P0 导致EKF收敛缓慢。

**修复**: 按状态语义分别设置：位置=0.01，速度=1.0，yaw=0.05，v_yaw=10.0，r=0.5，d_zc=0.1。

**影响文件**: `extended_kalman_filter.cpp`

#### 9. [低] EKF 更新后 yaw 未归一化

**问题**: yaw 残差归一化到 [-pi, pi]，但更新后的状态 `x_(6)` 未归一化。长时间运行后 yaw 可能漂移到很大的值，影响三角函数精度。

**修复**: 在 `update()` 后将 `x_(6)` 归一化到 [-pi, pi]。

**影响文件**: `extended_kalman_filter.cpp`

#### 10. [低] S.inverse() 无鲁棒处理

**问题**: 创新协方差矩阵 S 直接调用 `.inverse()`。当 S 接近奇异时可能产生数值错误。

**修复**: 改用 LLT 分解求逆，失败时回退到直接求逆。

**影响文件**: `extended_kalman_filter.cpp`

#### 11. [低] 飞行时间估算使用 pitch=0

**问题**: `armorsCallback()` 中估算弹丸飞行时间时传入 `pitch=0`，导致水平速度分量偏大（cos(0)=1），飞行时间偏小，延迟补偿不足。

**修复**: 用粗略的 pitch 估计值（atan2(y, horizontal_dist)）代替 0。

**影响文件**: `armor_solver_node.cpp`

#### 12. [低] selectBestArmor 未使用反陀螺角度阈值

**问题**: 声明了 `coming_angle_`、`leaving_angle_` 等反陀螺参数但未在 `selectBestArmor` 中使用，所有装甲板无论朝向都参与选择。

**修复**: 使用 `coming_angle_` 作为可见性过滤阈值，只选择在可见角度范围内的装甲板。无可见装甲板时回退到最正对的。

**影响文件**: `armor_solver_node.cpp`

#### 13. [低] 目标装甲板数量判定不完整

**问题**: `tracked_id_` 与 "sentry"/"outpost" 做字符串比较，但未覆盖数字ID "6"/"7"，也未处理工程 "2"。

**修复**: 新增 `updateTargetArmorsNum()` 方法，同时支持数字和名称两种ID格式。

**影响文件**: `armor_tracker.hpp`, `armor_tracker.cpp`

---

## 现有模型优缺点分析

### 检测模块 (传统视觉)

**优点**:
- 通道相减法对灯条高效且鲁棒，计算量小（无需GPU）
- 几何约束丰富（长宽比、角度、间距），误检率低
- 双判据颜色分类（RGB差值 + HSV色相回退），适应过曝/欠曝场景
- 所有参数可通过YAML动态配置

**缺点**:
- 无数字识别能力——所有装甲板输出 "unknown"，无法区分机器人编号，依赖外部分类器（当前缺失）
- 强光/高亮度干扰下（如场地照明灯、反光）可能产生误检测
- 通道相减法在红蓝灯条亮度接近时可能失效
- 不适应非标准装甲板（损坏、遮挡、倾斜严重的情况）
- 无法检测熄灯或部分灭灯的装甲板

### PnP解算

**优点**:
- IPPE_SQUARE 专为平面目标设计，两解中选重投影误差最小的，精度高
- 装甲板尺寸常量明确（133mm/227mm/56mm），模型点准确

**缺点**:
- 远距离时（>5m），灯条像素面积小，角点定位误差大，PnP解不稳定
- yaw 从旋转矩阵提取（atan2(R20, R00)），假设装甲板近似正对，大角度时退化
- 装甲板角点来自灯条top/bottom，受灯条提取噪声影响
- 未使用畸变校正后的角点，可能在画面边缘产生系统偏差

### 跟踪模块 (EKF)

**优点**:
- 10维状态向量同时建模旋转中心和装甲板关系，支持反陀螺预测
- 解析雅可比矩阵（非数值差分），计算快且无近似误差
- Joseph 形式协方差更新，数值稳定性好
- 完整的四状态状态机（LOST/DETECTING/TRACKING/TEMP_LOST），处理检测波动

**缺点**:
- 匀速+匀角速度运动模型——无法预测加速、变速运动，对突然启停、变向响应慢
- 单目标跟踪——只维护一个跟踪器，无法同时跟踪多个机器人
- 旋转模型假设固定半径 r 和均匀分布的装甲板——对非对称车型（如工程）不准确
- 无自适应噪声调整——Q/R 固定，无法根据运动状态动态调整（如静止时降低Q，高机动时增大Q）
- 状态向量中 yaw 与位置耦合——PnP 远距离 yaw 噪声大时会拖累位置估计

### 弹道补偿

**优点**:
- 含空气阻力模型（指数衰减），比纯抛物线更准确
- 迭代Newton-Raphson求解，收敛快（通常3-5次迭代）
- 支持手动补偿表，修正系统性偏差

**缺点**:
- 弹道下坠仅用 0.5*g*t^2（忽略竖直方向空气阻力），大仰角/远距离时偏差增大
- 空气阻力系数 k 为常量——未考虑弹丸速度衰减后阻力特性变化
- 弹速从串口反馈获取，存在通信延迟，高射频时弹速波动无法实时跟踪
- 未建模弹丸自旋（Magnus效应）和侧风影响

### 系统级

**优点**:
- ROS2 composable node 架构，零拷贝通信，低延迟
- 所有参数外部化（YAML），无需重编译即可调参
- 串口协议含心跳和自动重连，硬件层鲁棒

**缺点**:
- 缺少数字分类器（SVM/CNN），无法识别具体机器人编号
- 无多目标跟踪能力，面对多机器人时只能跟踪一个
- 延迟补偿为简单匀速外推——未使用更高阶预测（如加速度估计）
- 缺少单元测试和离线回放验证工具
- 参数调优依赖经验，无自动标定流程
