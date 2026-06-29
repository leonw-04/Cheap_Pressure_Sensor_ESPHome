#pragma once

namespace esphome {
namespace cheap_pressure_sensor {

enum RegisterAddress {
    REG_SLAVE_ADDRESS = 0x0000,
    REG_BAUDRATE = 0x0001,
    REG_UNIT = 0x0002,
    REG_DECIMAL_PLACES = 0x0003,
    REG_PV_SCALED = 0x0004,
    REG_ZERO_POINT = 0x0005,
    REG_FULL_SCALE = 0x0006,
    REG_OFFSET = 0x000C,
    REG_PV_FLOAT = 0x0016,
    REG_PARITY = 0x0025,
};

}  // namespace cheap_pressure_sensor
}  // namespace esphome
