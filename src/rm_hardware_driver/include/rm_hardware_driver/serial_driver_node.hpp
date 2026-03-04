#pragma once

#include <rclcpp/rclcpp.hpp>
#include <thread>
#include <atomic>
#include <string>
#include <mutex>
#include <chrono>

#include "rm_hardware_driver/fixed_packet.hpp"
#include "rm_interfaces/msg/gimbal_cmd.hpp"
#include "rm_interfaces/msg/serial_receive_data.hpp"

namespace rm_auto_aim {

/**
 * @brief 串口通信驱动节点
 *
 * 功能:
 * 1. 接收 GimbalCmd 消息，打包发送给下位机
 * 2. 从下位机接收数据，发布 SerialReceiveData 消息
 * 3. 心跳检测：定时检查是否收到下位机数据，超时则触发重连
 *
 * 协议: 固定长度包, Infantry协议
 */
class SerialDriverNode : public rclcpp::Node {
public:
    explicit SerialDriverNode(const rclcpp::NodeOptions& options);
    ~SerialDriverNode() override;

private:
    /**
     * @brief 打开串口
     */
    bool openPort();

    /**
     * @brief 关闭串口
     */
    void closePort();

    /**
     * @brief 尝试重连串口（由定时器触发）
     */
    void tryReconnect();

    /**
     * @brief 发送线程
     */
    void sendThread();

    /**
     * @brief 接收线程
     */
    void receiveThread();

    /**
     * @brief 云台命令回调
     */
    void gimbalCmdCallback(const rm_interfaces::msg::GimbalCmd::ConstSharedPtr& msg);

    /**
     * @brief 打包云台控制数据
     */
    Packet16 packGimbalCmd(double yaw, double pitch, bool fire);

    /**
     * @brief 解包接收数据
     */
    void unpackReceiveData(const Packet16& packet);

    /**
     * @brief 心跳检测（由定时器触发）
     * 检查是否在超时时间内收到过下位机数据，超时则关闭串口触发重连
     */
    void heartbeatCheck();

    // 串口参数
    std::string port_name_;
    int baud_rate_;
    int fd_ = -1;  // 串口文件描述符

    // 线程控制
    std::atomic<bool> running_{false};
    std::thread receive_thread_;

    // 最新的发送数据（线程安全）
    std::mutex send_mutex_;
    Packet16 send_packet_;
    bool has_new_data_ = false;

    // ROS接口
    rclcpp::Subscription<rm_interfaces::msg::GimbalCmd>::SharedPtr gimbal_cmd_sub_;
    rclcpp::Publisher<rm_interfaces::msg::SerialReceiveData>::SharedPtr receive_pub_;

    // 定时发送
    rclcpp::TimerBase::SharedPtr send_timer_;

    // 断线重连定时器
    rclcpp::TimerBase::SharedPtr reconnect_timer_;
    int reconnect_interval_ms_ = 1000;

    // 心跳检测
    rclcpp::TimerBase::SharedPtr heartbeat_timer_;
    int heartbeat_timeout_ms_ = 500;  // 心跳超时时间（毫秒）
    std::atomic<bool> heartbeat_alive_{false};  // 在超时周期内是否收到过数据
    std::chrono::steady_clock::time_point last_receive_time_;  // 上次收到数据的时间
    std::mutex heartbeat_mutex_;
};

}  // namespace rm_auto_aim
