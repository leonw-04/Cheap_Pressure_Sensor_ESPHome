#include "pressure_sensor.h"
#include "esphome/core/log.h"

namespace esphome {
namespace cheap_pressure_sensor {

static const char *const TAG = "cheap_pressure_sensor";

void CheapPressureSensor::setup() {
    ESP_LOGCONFIG(TAG, "Setting up Cheap Pressure Sensor (Slave ID: %u)...", slave_id_);
    if (this->parent_ == nullptr) {
        ESP_LOGE(TAG, "UART Parent is NULL! Check your configuration.");
    }
    parser_ = make_unique<ModbusParser>(slave_id_, [this](const ModbusFrame& frame) {
        this->on_frame(frame);
    });
}

void CheapPressureSensor::loop() {
    uint8_t byte;
    while (this->available()) {
        if (this->read_byte(&byte)) {
            ESP_LOGV(TAG, "UART read: 0x%02X", byte);
            parser_->feed(byte);
        } else {
            break;
        }
    }

    if (waiting_for_response_ && millis() - last_request_time_ > 3000) {
        ESP_LOGW(TAG, "Timed out waiting for response from slave %u (Last request %u ms ago)", slave_id_, (uint32_t)(millis() - last_request_time_));
        waiting_for_response_ = false;
    }
}

void CheapPressureSensor::update() {
    if (!config_fetched_) {
        ESP_LOGV(TAG, "Fetching configuration (Unit and Decimal Places)...");
        send_read_command(REG_UNIT, 2);
    } else {
        ESP_LOGV(TAG, "Updating sensor, sending read command for PV Float...");
        send_read_command(REG_PV_FLOAT, 2);
    }
}

void CheapPressureSensor::send_read_command(uint16_t start_address, uint16_t number_of_registers) {
    last_requested_reg_ = start_address;
    uint8_t frame[8];
    frame[0] = slave_id_;
    frame[1] = 0x03; // Read Holding Registers
    frame[2] = start_address >> 8;
    frame[3] = start_address & 0xFF;
    frame[4] = number_of_registers >> 8;
    frame[5] = number_of_registers & 0xFF;
    
    uint16_t crc = ModbusParser::calculate_crc(frame, 6);
    frame[6] = crc & 0xFF; // Low Byte
    frame[7] = crc >> 8;   // High Byte

    write_array(frame, 8);
    last_request_time_ = millis();
    waiting_for_response_ = true;
    ESP_LOGV(TAG, "Sent read command for address 0x%04X", start_address);
}

void CheapPressureSensor::on_frame(const ModbusFrame& frame) {
    waiting_for_response_ = false;
    
    if ((frame.function_code & 0x80) != 0) {
        ESP_LOGW(TAG, "Modbus Exception from slave %u: FC=0x%02X, Error=0x%02X", 
                 frame.slave_id, frame.function_code, frame.data[0]);
        return;
    }

    if (frame.function_code == 0x03) {
        if (last_requested_reg_ == REG_UNIT && frame.data.size() == 4) {
            // Antwort auf Unit (0x0002) und Decimal Places (0x0003)
            unit_ = (static_cast<uint16_t>(frame.data[0]) << 8) | frame.data[1];
            decimal_places_ = (static_cast<uint16_t>(frame.data[2]) << 8) | frame.data[3];
            config_fetched_ = true;
            
            ESP_LOGI(TAG, "Configuration loaded: Unit=%s (%u), Decimal Places=%u", 
                     unit_to_str(unit_).c_str(), unit_, decimal_places_);
            
            if (unit_sensor_ != nullptr) unit_sensor_->publish_state(unit_);
            if (decimal_places_sensor_ != nullptr) decimal_places_sensor_->publish_state(decimal_places_);
            if (unit_text_sensor_ != nullptr) unit_text_sensor_->publish_state(unit_to_str(unit_));
            
            // Sofort den ersten Messwert abfragen
            send_read_command(REG_PV_FLOAT, 2);
        } else if (last_requested_reg_ == REG_PV_FLOAT && frame.data.size() == 4) {
            // Antwort auf PV Float
            union {
                float f;
                uint8_t b[4];
            } data;
            
            // ABCD -> Byte 0 ist A (MSB), Byte 3 ist D (LSB)
            data.b[3] = frame.data[0]; // A
            data.b[2] = frame.data[1]; // B
            data.b[1] = frame.data[2]; // C
            data.b[0] = frame.data[3]; // D
            
            float pressure_raw = data.f;
            float pressure_bar = convert_to_bar(pressure_raw, unit_);
            
            ESP_LOGD(TAG, "Received Pressure: %.3f bar (Raw: %.3f %s)", 
                     pressure_bar, pressure_raw, unit_to_str(unit_).c_str());
            
            if (pressure_sensor_ != nullptr) {
                pressure_sensor_->publish_state(pressure_bar);
            }
        } else if (frame.data.size() == 2) {
            int16_t val = (static_cast<int16_t>(frame.data[0]) << 8) | frame.data[1];
            ESP_LOGV(TAG, "Received 2-byte data for reg 0x%04X: %d", last_requested_reg_, val);
        }
    }
}

float CheapPressureSensor::convert_to_bar(float value, uint8_t unit) {
    switch (unit) {
        case 0: return value * 10.0f;           // MPa -> bar
        case 1: return value * 0.01f;           // kPa -> bar
        case 2: return value * 0.00001f;        // Pa -> bar
        case 3: return value;                   // bar -> bar
        case 4: return value * 0.001f;          // mbar -> bar
        case 5: return value * 0.980665f;       // kg/cm² -> bar
        case 6: return value * 0.0689476f;      // psi -> bar
        case 16: return value * 0.0980665f;     // m (H2O) -> bar
        case 17: return value * 0.000980665f;   // cm (H2O) -> bar
        case 18: return value * 0.0000980665f;  // mm (H2O) -> bar
        default:
            ESP_LOGW(TAG, "Unknown unit %u, returning raw value", unit);
            return value;
    }
}

std::string CheapPressureSensor::unit_to_str(uint8_t unit) {
    switch (unit) {
        case 0: return "MPa";
        case 1: return "kPa";
        case 2: return "Pa";
        case 3: return "bar";
        case 4: return "mbar";
        case 5: return "kg/cm²";
        case 6: return "psi";
        case 16: return "m H2O";
        case 17: return "cm H2O";
        case 18: return "mm H2O";
        default: return "unknown";
    }
}

void CheapPressureSensor::dump_config() {
    ESP_LOGCONFIG(TAG, "Cheap Pressure Sensor:");
    ESP_LOGCONFIG(TAG, "  Slave ID: %u", slave_id_);
    if (config_fetched_) {
        ESP_LOGCONFIG(TAG, "  Unit: %s (%u)", unit_to_str(unit_).c_str(), unit_);
        ESP_LOGCONFIG(TAG, "  Decimal Places: %u", decimal_places_);
    } else {
        ESP_LOGCONFIG(TAG, "  Configuration not yet fetched from sensor");
    }
    LOG_SENSOR("  ", "Pressure", pressure_sensor_);
    LOG_SENSOR("  ", "Unit", unit_sensor_);
    LOG_SENSOR("  ", "Decimal Places", decimal_places_sensor_);
    LOG_TEXT_SENSOR("  ", "Unit Name", unit_text_sensor_);
    LOG_UPDATE_INTERVAL(this);
}

}  // namespace cheap_pressure_sensor
}  // namespace esphome
