#include "rm_hardware_driver/serial_driver_node.hpp"

#include <cmath>
#include <cstring>

// Linux串口头文件
#ifdef __linux__
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <errno.h>
#endif

namespace rm_auto_aim {

SerialDriverNode::SerialDriverNode(const rclcpp::NodeOptions& options)
    : Node("serial_driver", options)
{
    RCLCPP_INFO(get_logger(), "SerialDriverNode 初始化中...");

    // 声明参数
    this->declare_parameter("port_name", "/dev/ttyUSB0");
    this->declare_parameter("baud_rate", 115200);
    this->declare_parameter("enable_data_print", false);
    this->declare_parameter("heartbeat_timeout_ms", 500);

    port_name_ = this->get_parameter("port_name").as_string();
    baud_rate_ = this->get_parameter("baud_rate").as_int();
    heartbeat_timeout_ms_ = this->get_parameter("heartbeat_timeout_ms").as_int();

    // 订阅云台命令
    gimbal_cmd_sub_ = this->create_subscription<rm_interfaces::msg::GimbalCmd>(
        "/solver/gimbal_cmd", rclcpp::SensorDataQoS(),
        std::bind(&SerialDriverNode::gimbalCmdCallback, this, std::placeholders::_1));

    // 发布接收数据
    receive_pub_ = this->create_publisher<rm_interfaces::msg::SerialReceiveData>(
        "/serial/receive", rclcpp::SensorDataQoS());

    // 打开串口
    running_ = true;
    if (openPort()) {
        // 初始化心跳时间戳
        {
            std::lock_guard<std::mutex> lock(heartbeat_mutex_);
            last_receive_time_ = std::chrono::steady_clock::now();
        }
        heartbeat_alive_ = true;
        // 启动接收线程
        receive_thread_ = std::thread(&SerialDriverNode::receiveThread, this);
        // 定时发送 (1kHz)
        send_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(1),
            std::bind(&SerialDriverNode::sendThread, this));
        RCLCPP_INFO(get_logger(), "串口 %s 已打开", port_name_.c_str());
    } else {
        RCLCPP_WARN(get_logger(), "无法打开串口 %s，将定期尝试重连", port_name_.c_str());
    }

    // 启动断线重连定时器（无论初始是否连接成功都启动）
    reconnect_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(reconnect_interval_ms_),
        std::bind(&SerialDriverNode::tryReconnect, this));

    // 启动心跳检测定时器
    heartbeat_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(heartbeat_timeout_ms_),
        std::bind(&SerialDriverNode::heartbeatCheck, this));

    RCLCPP_INFO(get_logger(), "SerialDriverNode 初始化完成 (心跳超时: %dms)", heartbeat_timeout_ms_);
}

SerialDriverNode::~SerialDriverNode() {
    running_ = false;
    if (receive_thread_.joinable()) {
        receive_thread_.join();
    }
    closePort();
}

bool SerialDriverNode::openPort() {
#ifdef __linux__
    fd_ = open(port_name_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd_ < 0) {
        RCLCPP_ERROR(get_logger(), "打开串口失败: %s", strerror(errno));
        return false;
    }

    // 配置串口
    struct termios tty;
    memset(&tty, 0, sizeof(tty));
    if (tcgetattr(fd_, &tty) != 0) {
        RCLCPP_ERROR(get_logger(), "获取串口属性失败");
        close(fd_);
        fd_ = -1;
        return false;
    }

    // 波特率
    speed_t speed;
    switch (baud_rate_) {
        case 9600: speed = B9600; break;
        case 115200: speed = B115200; break;
        case 460800: speed = B460800; break;
        case 921600: speed = B921600; break;
        default: speed = B115200; break;
    }
    cfsetospeed(&tty, speed);
    cfsetispeed(&tty, speed);

    // 8N1, 无流控
    tty.c_cflag &= ~PARENB;    // 无校验
    tty.c_cflag &= ~CSTOPB;    // 1位停止位
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;         // 8位数据位
    tty.c_cflag &= ~CRTSCTS;   // 无硬件流控
    tty.c_cflag |= CREAD | CLOCAL;

    // 原始模式
    tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    tty.c_iflag &= ~(IXON | IXOFF | IXANY | ICRNL);
    tty.c_oflag &= ~OPOST;

    // 读取超时
    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 1;  // 100ms超时

    if (tcsetattr(fd_, TCSANOW, &tty) != 0) {
        RCLCPP_ERROR(get_logger(), "设置串口属性失败");
        close(fd_);
        fd_ = -1;
        return false;
    }

    tcflush(fd_, TCIOFLUSH);
    return true;
#else
    RCLCPP_WARN(get_logger(), "串口通信仅支持Linux平台，当前使用虚拟模式");
    return false;
#endif
}

void SerialDriverNode::closePort() {
#ifdef __linux__
    if (fd_ >= 0) {
        close(fd_);
        fd_ = -1;
    }
#endif
}

void SerialDriverNode::gimbalCmdCallback(
    const rm_interfaces::msg::GimbalCmd::ConstSharedPtr& msg)
{
    std::lock_guard<std::mutex> lock(send_mutex_);
    send_packet_ = packGimbalCmd(msg->yaw, msg->pitch, msg->fire);
    has_new_data_ = true;
}

Packet16 SerialDriverNode::packGimbalCmd(double yaw, double pitch, bool fire) {
    Packet16 packet;

    // 将浮点数转为int16 (放大100倍传输)
    auto yaw_int = static_cast<int16_t>(yaw * 180.0 / M_PI * 100);
    auto pitch_int = static_cast<int16_t>(pitch * 180.0 / M_PI * 100);
    uint8_t fire_byte = fire ? 1 : 0;

    // 打包: [header][yaw_h][yaw_l][pitch_h][pitch_l][fire]...[checksum][tail]
    packet.load<int16_t>(1, yaw_int);
    packet.load<int16_t>(3, pitch_int);
    packet.load<uint8_t>(5, fire_byte);
    packet.setChecksum();

    return packet;
}

void SerialDriverNode::tryReconnect() {
#ifdef __linux__
    if (fd_ >= 0) return;  // 串口已连接，无需重连

    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 3000,
                         "尝试重连串口 %s ...", port_name_.c_str());

    if (openPort()) {
        RCLCPP_INFO(get_logger(), "串口 %s 重连成功", port_name_.c_str());
        // 重连成功，重置心跳状态
        {
            std::lock_guard<std::mutex> lock(heartbeat_mutex_);
            last_receive_time_ = std::chrono::steady_clock::now();
        }
        heartbeat_alive_ = true;
        // 如果接收线程还没启动或已退出，重新启动
        if (!receive_thread_.joinable()) {
            receive_thread_ = std::thread(&SerialDriverNode::receiveThread, this);
        }
        // 如果发送定时器还没创建，创建它
        if (!send_timer_) {
            send_timer_ = this->create_wall_timer(
                std::chrono::milliseconds(1),
                std::bind(&SerialDriverNode::sendThread, this));
        }
    }
#endif
}

void SerialDriverNode::sendThread() {
#ifdef __linux__
    if (fd_ < 0) return;

    std::lock_guard<std::mutex> lock(send_mutex_);
    if (!has_new_data_) return;

    ssize_t written = write(fd_, send_packet_.data.data(), Packet16::SIZE);
    if (written < 0) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                             "串口发送失败: %s，将触发重连", strerror(errno));
        // 关闭失效的文件描述符，让重连定时器处理
        closePort();
    }
    has_new_data_ = false;
#endif
}

void SerialDriverNode::receiveThread() {
#ifdef __linux__
    std::array<uint8_t, 256> buffer;
    std::array<uint8_t, Packet16::SIZE> packet_buffer;
    size_t packet_idx = 0;

    while (running_) {
        if (fd_ < 0) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            continue;
        }

        ssize_t bytes_read = read(fd_, buffer.data(), buffer.size());
        if (bytes_read < 0 && errno != EAGAIN && errno != EWOULDBLOCK) {
            // 串口读取错误，可能断线
            RCLCPP_WARN(get_logger(), "串口读取错误: %s，关闭端口等待重连", strerror(errno));
            closePort();
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            continue;
        }
        if (bytes_read <= 0) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            continue;
        }

        // 逐字节解析，查找完整包
        for (ssize_t i = 0; i < bytes_read; i++) {
            uint8_t byte = buffer[i];

            if (packet_idx == 0 && byte != Packet16::HEADER) {
                continue;  // 等待帧头
            }

            packet_buffer[packet_idx++] = byte;

            if (packet_idx >= Packet16::SIZE) {
                // 收到完整包
                Packet16 packet;
                std::memcpy(packet.data.data(), packet_buffer.data(), Packet16::SIZE);

                if (packet.isValid()) {
                    unpackReceiveData(packet);
                }

                packet_idx = 0;
            }
        }
    }
#endif
}

void SerialDriverNode::unpackReceiveData(const Packet16& packet) {
    // 更新心跳时间戳：收到有效数据包，说明下位机在线
    {
        std::lock_guard<std::mutex> lock(heartbeat_mutex_);
        last_receive_time_ = std::chrono::steady_clock::now();
    }
    heartbeat_alive_ = true;

    rm_interfaces::msg::SerialReceiveData msg;
    msg.header.stamp = this->now();

    // 解包: [header][bullet_speed(f32)][cur_yaw(i16)][cur_pitch(i16)][color][mode]...
    // 弹速 (float, 4字节)
    msg.bullet_speed = static_cast<double>(packet.read<float>(1));

    // 当前yaw/pitch (int16, 放大了100倍)
    auto cur_yaw_int = packet.read<int16_t>(5);
    auto cur_pitch_int = packet.read<int16_t>(7);
    msg.cur_yaw = static_cast<double>(cur_yaw_int) / 100.0 * M_PI / 180.0;
    msg.cur_pitch = static_cast<double>(cur_pitch_int) / 100.0 * M_PI / 180.0;

    // 颜色和模式
    msg.color = packet.read<uint8_t>(9);
    msg.mode = packet.read<uint8_t>(10);

    receive_pub_->publish(msg);
}

void SerialDriverNode::heartbeatCheck() {
#ifdef __linux__
    if (fd_ < 0) return;  // 串口未连接，由重连定时器处理

    std::chrono::steady_clock::time_point last_time;
    {
        std::lock_guard<std::mutex> lock(heartbeat_mutex_);
        last_time = last_receive_time_;
    }

    auto now = std::chrono::steady_clock::now();
    auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_time).count();

    if (elapsed_ms > heartbeat_timeout_ms_) {
        if (heartbeat_alive_) {
            // 首次检测到心跳丢失
            heartbeat_alive_ = false;
            RCLCPP_WARN(get_logger(),
                "心跳丢失! 已超过 %dms 未收到下位机数据 (超时阈值: %dms)，关闭串口等待重连",
                static_cast<int>(elapsed_ms), heartbeat_timeout_ms_);
            closePort();
        }
    }
#endif
}

}  // namespace rm_auto_aim

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(rm_auto_aim::SerialDriverNode)
