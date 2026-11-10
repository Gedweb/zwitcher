#ifndef ZWITCHER_OTA_ELEMENT_PARSER_H
#define ZWITCHER_OTA_ELEMENT_PARSER_H

#include <stdint.h>

// The stack removes the outer OTA header, but sub-element bytes can span blocks.
class OtaElementParser {
public:
    bool consume(uint32_t total_size, const uint8_t *payload, uint16_t payload_size,
                 const uint8_t **data, uint16_t *length) {
        *data = nullptr;
        *length = 0;
        if (failed || total_size <= sizeof(header) ||
            (expected_size && expected_size != total_size) ||
            (payload_size && !payload) || received > total_size ||
            payload_size > total_size - received) {
            failed = true;
            return false;
        }
        expected_size = total_size;
        received += payload_size;
        while (header_size < sizeof(header) && payload_size) {
            header[header_size++] = *payload++;
            --payload_size;
        }
        if (header_size < sizeof(header)) {
            return true;
        }
        if (!parsed) {
            uint16_t tag = uint16_t(header[0]) | (uint16_t(header[1]) << 8);
            uint32_t size = uint32_t(header[2]) | (uint32_t(header[3]) << 8) |
                            (uint32_t(header[4]) << 16) | (uint32_t(header[5]) << 24);
            if (tag != 0 || size != total_size - sizeof(header)) {
                failed = true;
                return false;
            }
            parsed = true;
        }
        *data = payload;
        *length = payload_size;
        return true;
    }

    bool complete() const {
        return !failed && parsed && received == expected_size;
    }

private:
    uint8_t header[6] = {};
    uint8_t header_size = 0;
    uint32_t expected_size = 0;
    uint32_t received = 0;
    bool parsed = false;
    bool failed = false;
};

#endif
