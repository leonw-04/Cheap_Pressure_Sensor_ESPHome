#pragma once

#include "esphome/core/component.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/text_sensor/text_sensor.h"
#include "esphome/components/uart/uart.h"
#include "modbus_parser.h"
#include "register_map.h"

namespace esphome {
namespace cheap_pressure_sensor {

class CheapPressureSensor : public PollingComponent, public uart::UARTDevice {
public:
    void set_pressure_sensor(sensor::Sensor *pressure_sensor) { pressure_sensor_ = pressure_sensor; }
    void set_unit_sensor(sensor::Sensor *unit_sensor) { unit_sensor_ = unit_sensor; }
    void set_decimal_places_sensor(sensor::Sensor *decimal_places_sensor) { decimal_places_sensor_ = decimal_places_sensor; }
    void set_unit_text_sensor(text_sensor::TextSensor *unit_text_sensor) { unit_text_sensor_ = unit_text_sensor; }
    void set_slave_id(uint8_t slave_id) { slave_id_ = slave_id; }

    void setup() override;
    void loop() override;
    void update() override;
    void dump_config() override;

protected:
    void on_frame(const ModbusFrame& frame);
    void send_read_command(uint16_t start_address, uint16_t number_of_registers);
    float convert_to_bar(float value, uint8_t unit);
    std::string unit_to_str(uint8_t unit);

    sensor::Sensor *pressure_sensor_{nullptr};
    sensor::Sensor *unit_sensor_{nullptr};
    sensor::Sensor *decimal_places_sensor_{nullptr};
    text_sensor::TextSensor *unit_text_sensor_{nullptr};
    uint8_t slave_id_{1};
    std::unique_ptr<ModbusParser> parser_;
    uint32_t last_request_time_{0};
    uint16_t last_requested_reg_{0};
    bool waiting_for_response_{false};
    bool config_fetched_{false};
    uint8_t unit_{3}; // Default to bar
    uint8_t decimal_places_{0};
};

}  // namespace cheap_pressure_sensor
}  // namespace esphome
