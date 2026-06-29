#include "modbus_parser.h"
#include "esphome/core/log.h"

namespace esphome {
namespace cheap_pressure_sensor {

static const char *const TAG = "cheap_pressure_sensor";

void ModbusParser::feed(uint8_t byte) {
    ESP_LOGVV(TAG, "Parser feed: 0x%02X", byte);
    buffer_.push_back(byte);
    process_buffer();
}

void ModbusParser::feed(const uint8_t* data, size_t len) {
    buffer_.insert(buffer_.end(), data, data + len);
    process_buffer();
}

void ModbusParser::process_buffer() {
    if (!buffer_.empty()) {
        ESP_LOGVV(TAG, "Process buffer, size: %zu", buffer_.size());
    }
    // Ein Modbus RTU Frame hat mindestens 5 Bytes: Addr(1) + FC(1) + Data(min 1) + CRC(2)
    while (buffer_.size() >= 5) {
        // Sliding window: Suche nach slave_id an Position 0
        if (buffer_[0] != slave_id_) {
            ESP_LOGI(TAG, "Parser: skipping noise byte 0x%02X (waiting for 0x%02X)", buffer_[0], slave_id_);
            buffer_.erase(buffer_.begin());
            continue;
        }

        uint8_t fc = buffer_[1];
        ESP_LOGI(TAG, "Parser: found slave_id 0x%02X, FC=0x%02X candidate", buffer_[0], fc);
        // Erlaubte Function Codes laut Issue + Exception Codes (FC | 0x80)
        bool is_exception = (fc & 0x80) != 0;
        uint8_t base_fc = fc & 0x7F;

        if (fc != 0x03 && fc != 0x04 && fc != 0x06 && fc != 0x10 &&
            base_fc != 0x03 && base_fc != 0x04 && base_fc != 0x06 && base_fc != 0x10) {
            ESP_LOGI(TAG, "Parser: invalid FC 0x%02X for candidate, skipping slave_id byte", fc);
            buffer_.erase(buffer_.begin());
            continue;
        }

        size_t expected_len = 0;
        // ... (rest of logic)
        if (is_exception) {
            expected_len = 5;
        } else if (fc == 0x03 || fc == 0x04) {
            if (buffer_.size() < 3) {
                ESP_LOGI(TAG, "Parser: wait for byte count byte");
                break;
            }
            uint8_t byte_count = buffer_[2];
            expected_len = 5 + byte_count;
        } else if (fc == 0x06) {
            expected_len = 8;
        } else if (fc == 0x10) {
            if (buffer_.size() < 7) {
                ESP_LOGI(TAG, "Parser: wait for write byte count byte");
                break;
            }
            uint8_t byte_count = buffer_[6];
            expected_len = 9 + byte_count;
        }

        // Haben wir genug Bytes für diesen Frame-Typ?
        if (buffer_.size() < expected_len) {
            ESP_LOGI(TAG, "Parser: frame candidate (FC=0x%02X) needs %zu bytes, have %zu. Waiting...", fc, expected_len, buffer_.size());
            break;
        }

        // CRC Prüfung
        uint16_t received_crc = (static_cast<uint16_t>(buffer_[expected_len - 1]) << 8) | buffer_[expected_len - 2];
        uint16_t calculated_crc = calculate_crc(buffer_.data(), expected_len - 2);

        if (received_crc == calculated_crc) {
            ESP_LOGI(TAG, "Parser: VALID Modbus frame received (FC=0x%02X, len=%zu)", fc, expected_len);
            // ... (rest)
            ModbusFrame frame;
            frame.slave_id = buffer_[0];
            frame.function_code = buffer_[1];
            frame.raw.assign(buffer_.begin(), buffer_.begin() + expected_len);
            
            if (fc == 0x03 || fc == 0x04) {
                frame.data.assign(buffer_.begin() + 3, buffer_.begin() + expected_len - 2);
            } else {
                frame.data.assign(buffer_.begin() + 2, buffer_.begin() + expected_len - 2);
            }
            frame.crc = received_crc;

            callback_(frame);
            
            // Frame verarbeitet, aus Buffer löschen
            buffer_.erase(buffer_.begin(), buffer_.begin() + expected_len);
        } else {
            ESP_LOGW(TAG, "Parser: CRC Check failed! Received: 0x%04X, Calculated: 0x%04X", received_crc, calculated_crc);
            // CRC falsch. Vielleicht war das erste Byte zufällig die Slave ID.
            // Wir löschen das erste Byte und suchen weiter.
            buffer_.erase(buffer_.begin());
        }
    }
}

uint16_t ModbusParser::calculate_crc(const uint8_t* data, size_t len) {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (int j = 0; j < 8; j++) {
            if (crc & 0x0001) {
                crc >>= 1;
                crc ^= 0xA001;
            } else {
                crc >>= 1;
            }
        }
    }
    return crc;
}

}  // namespace cheap_pressure_sensor
}  // namespace esphome
