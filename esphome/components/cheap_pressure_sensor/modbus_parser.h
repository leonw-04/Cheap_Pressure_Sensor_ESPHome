#pragma once

#include <vector>
#include <cstdint>
#include <functional>

namespace esphome {
namespace cheap_pressure_sensor {

struct ModbusFrame {
    uint8_t slave_id;
    uint8_t function_code;
    std::vector<uint8_t> data;
    uint16_t crc;
    std::vector<uint8_t> raw;
};

class ModbusParser {
public:
    using FrameCallback = std::function<void(const ModbusFrame&)>;

    ModbusParser(uint8_t slave_id, FrameCallback callback)
        : slave_id_(slave_id), callback_(callback) {}

    void feed(uint8_t byte);
    void feed(const uint8_t* data, size_t len);
    
    static uint16_t calculate_crc(const uint8_t* data, size_t len);

private:
    uint8_t slave_id_;
    FrameCallback callback_;
    std::vector<uint8_t> buffer_;

    void process_buffer();
};

}  // namespace cheap_pressure_sensor
}  // namespace esphome
