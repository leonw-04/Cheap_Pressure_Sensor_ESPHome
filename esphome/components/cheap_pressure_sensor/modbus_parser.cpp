#include "modbus_parser.h"

namespace esphome {
namespace cheap_pressure_sensor {

void ModbusParser::feed(uint8_t byte) {
    buffer_.push_back(byte);
    process_buffer();
}

void ModbusParser::feed(const uint8_t* data, size_t len) {
    buffer_.insert(buffer_.end(), data, data + len);
    process_buffer();
}

void ModbusParser::process_buffer() {
    // Ein Modbus RTU Frame hat mindestens 5 Bytes: Addr(1) + FC(1) + Data(min 1) + CRC(2)
    while (buffer_.size() >= 5) {
        // Sliding window: Suche nach slave_id an Position 0
        if (buffer_[0] != slave_id_) {
            buffer_.erase(buffer_.begin());
            continue;
        }

        uint8_t fc = buffer_[1];
        // Erlaubte Function Codes laut Issue
        if (fc != 0x03 && fc != 0x04 && fc != 0x06 && fc != 0x10) {
            buffer_.erase(buffer_.begin());
            continue;
        }

        size_t expected_len = 0;
        if (fc == 0x03 || fc == 0x04) {
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
        if (buffer_.size() < expected_len) break;

        // CRC Prüfung (Little Endian bei Modbus RTU)
        uint16_t received_crc = (static_cast<uint16_t>(buffer_[expected_len - 1]) << 8) | buffer_[expected_len - 2];
        uint16_t calculated_crc = calculate_crc(buffer_.data(), expected_len - 2);

        if (received_crc == calculated_crc) {
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
