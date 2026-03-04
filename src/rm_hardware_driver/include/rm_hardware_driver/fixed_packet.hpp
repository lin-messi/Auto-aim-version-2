#pragma once

#include <array>
#include <cstdint>
#include <cstring>

namespace rm_auto_aim {

/**
 * @brief 固定长度串口通信数据包
 *
 * 帧格式: [0xFF] [payload...] [checksum] [0x0D]
 * @tparam N 数据包总长度(字节)
 */
template <size_t N>
struct FixedPacket {
    static constexpr uint8_t HEADER = 0xFF;
    static constexpr uint8_t TAIL = 0x0D;
    static constexpr size_t SIZE = N;

    std::array<uint8_t, N> data{};

    FixedPacket() {
        data.fill(0);
        data[0] = HEADER;
        data[N - 1] = TAIL;
    }

    /**
     * @brief 从指定偏移位置加载数据
     */
    template <typename T>
    void load(size_t offset, T value) {
        static_assert(std::is_trivially_copyable_v<T>);
        if (offset + sizeof(T) < N - 1) {
            std::memcpy(&data[offset], &value, sizeof(T));
        }
    }

    /**
     * @brief 从指定偏移位置读取数据
     */
    template <typename T>
    T read(size_t offset) const {
        static_assert(std::is_trivially_copyable_v<T>);
        T value{};
        if (offset + sizeof(T) < N - 1) {
            std::memcpy(&value, &data[offset], sizeof(T));
        }
        return value;
    }

    /**
     * @brief 计算校验和 (XOR)
     */
    uint8_t calcChecksum() const {
        uint8_t checksum = 0;
        for (size_t i = 1; i < N - 2; i++) {
            checksum ^= data[i];
        }
        return checksum;
    }

    /**
     * @brief 设置校验和
     */
    void setChecksum() {
        data[N - 2] = calcChecksum();
    }

    /**
     * @brief 验证包有效性
     */
    bool isValid() const {
        return data[0] == HEADER &&
               data[N - 1] == TAIL &&
               data[N - 2] == calcChecksum();
    }
};

// 常用包大小
using Packet16 = FixedPacket<16>;
using Packet32 = FixedPacket<32>;

}  // namespace rm_auto_aim
