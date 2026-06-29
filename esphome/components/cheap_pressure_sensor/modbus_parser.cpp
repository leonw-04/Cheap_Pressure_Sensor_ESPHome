#include "modbus_parser.h"
#include "esphome/core/log.h"

namespace esphome {
namespace cheap_pressure_sensor {

static const char *const TAG = "cheap_pressure_sensor.parser";

void ModbusParser::feed(uint8_t byte) {
    // ESP_LOGD(TAG, "Parser received byte: 0x%02X", byte); // Zu viel Output, da wir es schon in loop() loggen
    buffer_.push_back(byte);
    process_buffer();
}

void ModbusParser::feed(const uint8_t* data, size_t len) {
    buffer_.insert(buffer_.end(), data, data + len);
    process_buffer();
}

void ModbusParser::process_buffer() {
    // ESP_LOGD(TAG, "Processing buffer, size: %zu", buffer_.size());
    // Ein Modbus RTU Frame hat mindestens 5 Bytes: Addr(1) + FC(1) + Data(min 1) + CRC(2)
    while (buffer_.size() >= 5) {
        // Sliding window: Suche nach slave_id an Position 0
        if (buffer_[0] != slave_id_) {
            ESP_LOGD(TAG, "Skipping noise byte: 0x%02X", buffer_[0]);
            buffer_.erase(buffer_.begin());
            continue;
        }

        uint8_t fc = buffer_[1];
        // Erlaubte Function Codes laut Issue + Exception Codes (FC | 0x80)
        bool is_exception = (fc & 0x80) != 0;
        uint8_t base_fc = fc & 0x7F;

        if (fc != 0x03 && fc != 0x04 && fc != 0x06 && fc != 0x10 &&
            base_fc != 0x03 && base_fc != 0x04 && base_fc != 0x06 && base_fc != 0x10) {
            ESP_LOGD(TAG, "Found slave_id but invalid FC: 0x%02X. Skipping slave_id byte.", fc);
            buffer_.erase(buffer_.begin());
            continue;
        }

        size_t expected_len = 0;
        if (is_exception) {
            // Exception: Addr(1) + FC|80(1) + ErrorCode(1) + CRC(2)
            expected_len = 5;
        } else if (fc == 0x03 || fc == 0x04) {
            // Read: Addr(1) + FC(1) + ByteCount(1) + Data(N) + CRC(2)
            if (buffer_.size() < 3) break; // Brauchen ByteCount
            uint8_t byte_count = buffer_[2];
            expected_len = 5 + byte_count;
        } else if (fc == 0x06) {
            // Write Single: Addr(1) + FC(1) + RegAddr(2) + Value(2) + CRC(2)
            expected_len = 8;
        } else if (fc == 0x10) {
            // Write Multiple: Addr(1) + FC(1) + RegAddr(2) + RegCount(2) + ByteCount(1) + Data(N) + CRC(2)
            if (buffer_.size() < 7) break; // Brauchen ByteCount
            uint8_t byte_count = buffer_[6];
            expected_len = 9 + byte_count;
        }

        // Haben wir genug Bytes für diesen Frame-Typ?
        if (buffer_.size() < expected_len) {
            ESP_LOGD(TAG, "Waiting for more bytes. Have %zu, need %zu", buffer_.size(), expected_len);
            break;
        }

        // CRC Prüfung (Little Endian bei Modbus RTU)
        uint16_t received_crc = (static_cast<uint16_t>(buffer_[expected_len - 1]) << 8) | buffer_[expected_len - 2];
        uint16_t calculated_crc = calculate_crc(buffer_.data(), expected_len - 2);

        if (received_crc == calculated_crc) {
            ESP_LOGD(TAG, "Valid Modbus frame received (FC=0x%02X, len=%zu)", fc, expected_len);
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
            ESP_LOGD(TAG, "CRC Check failed! Received: 0x%04X, Calculated: 0x%04X", received_crc, calculated_crc);
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
