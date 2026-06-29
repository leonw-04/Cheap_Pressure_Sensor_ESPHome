#pragma once

#include "esphome/core/component.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/uart/uart.h"
#include "modbus_parser.h"
#include "register_map.h"

namespace esphome {
namespace cheap_pressure_sensor {

class CheapPressureSensor : public PollingComponent, public uart::UARTDevice {
public:
    void set_pressure_sensor(sensor::Sensor *pressure_sensor) { pressure_sensor_ = pressure_sensor; }
    void set_slave_id(uint8_t slave_id) { slave_id_ = slave_id; }

    void setup() override;
    void loop() override;
    void update() override;
    void dump_config() override;

protected:
    void on_frame(const ModbusFrame& frame);
    void send_read_command(uint16_t start_address, uint16_t number_of_registers);

    sensor::Sensor *pressure_sensor_{nullptr};
    uint8_t slave_id_{1};
    std::unique_ptr<ModbusParser> parser_;
    uint32_t last_request_time_{0};
    bool waiting_for_response_{false};
};

}  // namespace cheap_pressure_sensor
}  // namespace esphome
