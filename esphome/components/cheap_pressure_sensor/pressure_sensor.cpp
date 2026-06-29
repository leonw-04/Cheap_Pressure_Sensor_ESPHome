#include "pressure_sensor.h"
#include "esphome/core/log.h"

namespace esphome {
namespace cheap_pressure_sensor {

static const char *const TAG = "cheap_pressure_sensor";

void CheapPressureSensor::setup() {
    ESP_LOGCONFIG(TAG, "Setting up Cheap Pressure Sensor with Slave ID %u...", slave_id_);
    parser_ = make_unique<ModbusParser>(slave_id_, [this](const ModbusFrame& frame) {
        this->on_frame(frame);
    });
}

void CheapPressureSensor::loop() {
    uint32_t bytes_read = 0;
    while (available()) {
        uint8_t byte;
        read_byte(&byte);
        parser_->feed(byte);
        bytes_read++;
    }
    if (bytes_read > 0) {
        ESP_LOGD(TAG, "Read %u bytes from UART", bytes_read);
    }

    if (waiting_for_response_ && millis() - last_request_time_ > 2000) {
        ESP_LOGW(TAG, "Timed out waiting for response from slave %u", slave_id_);
        waiting_for_response_ = false;
    }
}

void CheapPressureSensor::update() {
    // Abfrage des PV Float Werts (Register 0x0016, Länge 2 Words = 4 Bytes)
    // Wir nutzen FC 0x03 (Read Holding Registers)
    send_read_command(REG_PV_FLOAT, 2);
}

void CheapPressureSensor::send_read_command(uint16_t start_address, uint16_t number_of_registers) {
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

    // Wir erwarten eine Antwort auf unsere Read-Anfrage (FC 0x03)
    if (frame.function_code == 0x03) {
        if (frame.data.size() == 4) {
            // Dies ist höchstwahrscheinlich die Antwort auf den PV Float (4 Bytes)
            // Byte Order Float: ABCD (Big Endian IEEE754)
            // Auf ESP32/ESP8266 (Little Endian CPU) müssen wir die Bytes umkehren
            union {
                float f;
                uint8_t b[4];
            } data;
            
            // ABCD -> Byte 0 ist A (MSB), Byte 3 ist D (LSB)
            // In Little Endian Memory: b[0]=D, b[1]=C, b[2]=B, b[3]=A
            data.b[3] = frame.data[0]; // A
            data.b[2] = frame.data[1]; // B
            data.b[1] = frame.data[2]; // C
            data.b[0] = frame.data[3]; // D
            
            ESP_LOGD(TAG, "Received PV Float: %.3f", data.f);
            if (pressure_sensor_ != nullptr) {
                pressure_sensor_->publish_state(data.f);
            }
        } else if (frame.data.size() == 2) {
            // Wahrscheinlich PV Scaled (int16), falls wir das später abfragen
            int16_t val = (static_cast<int16_t>(frame.data[0]) << 8) | frame.data[1];
            ESP_LOGD(TAG, "Received PV Scaled: %d", val);
        }
    }
}

void CheapPressureSensor::dump_config() {
    ESP_LOGCONFIG(TAG, "Cheap Pressure Sensor:");
    ESP_LOGCONFIG(TAG, "  Slave ID: %u", slave_id_);
    LOG_SENSOR("  ", "Pressure", pressure_sensor_);
    LOG_UPDATE_INTERVAL(this);
}

}  // namespace cheap_pressure_sensor
}  // namespace esphome
