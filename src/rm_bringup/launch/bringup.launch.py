import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    TimerAction,
    GroupAction,
)
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import (
    ComposableNodeContainer,
    Node,
    PushRosNamespace,
)
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    """生成自瞄系统完整启动描述"""

    # ===== 包路径 =====
    bringup_dir = get_package_share_directory('rm_bringup')
    config_dir = os.path.join(bringup_dir, 'config')
    params_dir = os.path.join(config_dir, 'node_params')

    # ===== 参数文件路径 =====
    detector_params = os.path.join(params_dir, 'armor_detector_params.yaml')
    solver_params = os.path.join(params_dir, 'armor_solver_params.yaml')
    camera_params = os.path.join(params_dir, 'camera_driver_params.yaml')
    serial_params = os.path.join(params_dir, 'serial_driver_params.yaml')

    # ===== 启动参数 =====
    namespace_arg = DeclareLaunchArgument(
        'namespace', default_value='',
        description='ROS2 namespace'
    )
    debug_arg = DeclareLaunchArgument(
        'debug', default_value='true',
        description='Enable debug mode'
    )

    # ===== 核心节点容器 (组件化，共享进程，零拷贝通信) =====
    auto_aim_container = ComposableNodeContainer(
        name='auto_aim_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',  # 多线程容器
        composable_node_descriptions=[
            # 相机驱动
            ComposableNode(
                package='rm_hardware_driver',
                plugin='rm_auto_aim::CameraDriverNode',
                name='camera_driver',
                parameters=[camera_params],
            ),
            # 装甲板检测器
            ComposableNode(
                package='rm_auto_aim',
                plugin='rm_auto_aim::ArmorDetectorNode',
                name='armor_detector',
                parameters=[detector_params],
            ),
            # 装甲板解算器
            ComposableNode(
                package='rm_auto_aim',
                plugin='rm_auto_aim::ArmorSolverNode',
                name='armor_solver',
                parameters=[solver_params],
            ),
        ],
        output='screen',
    )

    # ===== 串口驱动 (延迟1.5s启动) =====
    serial_driver = TimerAction(
        period=1.5,
        actions=[
            Node(
                package='rm_hardware_driver',
                executable='serial_driver_node',
                name='serial_driver',
                parameters=[serial_params],
                output='screen',
            ),
        ],
    )

    # ===== 组合启动 =====
    auto_aim_group = GroupAction(
        actions=[
            PushRosNamespace(LaunchConfiguration('namespace')),
            # 先启动核心容器
            TimerAction(
                period=0.5,
                actions=[auto_aim_container],
            ),
            # 延迟启动串口
            serial_driver,
        ]
    )

    return LaunchDescription([
        namespace_arg,
        debug_arg,
        auto_aim_group,
    ])
