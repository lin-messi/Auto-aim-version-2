# RoboMaster Auto-Aim System (v2)

基于传统视觉 + EKF 的 RoboMaster 自瞄系统，使用 ROS2 构建。

## 编译运行
cd ~/auto-aim
colcon build --symlink-install
source install/setup.bash
ros2 launch rm_bringup bringup.launch.py



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
