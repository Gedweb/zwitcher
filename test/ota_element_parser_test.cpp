#include "../src/ota_element_parser.h"
#include <assert.h>
#include <stdio.h>
#include <vector>

int main() {
    const uint8_t image[] = {0, 0, 4, 0, 0, 0, 0xe9, 1, 2, 3};
    // Every possible partition into nonempty blocks, including 1-byte headers
    // and a block ending exactly at the six-byte header boundary.
    for (unsigned boundaries = 0; boundaries < (1u << (sizeof(image) - 1)); ++boundaries) {
        OtaElementParser parser;
        std::vector<uint8_t> firmware;
        unsigned start = 0;
        for (unsigned end = 1; end <= sizeof(image); ++end) {
            if (end != sizeof(image) && !(boundaries & (1u << (end - 1)))) {
                continue;
            }
            const uint8_t *data;
            uint16_t length;
            assert(parser.consume(sizeof(image), image + start, end - start, &data, &length));
            if (length) {
                firmware.insert(firmware.end(), data, data + length);
            }
            assert(parser.complete() == (end == sizeof(image)));
            start = end;
        }
        assert((firmware == std::vector<uint8_t>{0xe9, 1, 2, 3}));
    }

    const uint8_t *data;
    uint16_t length;
    OtaElementParser parser;
    assert(!parser.complete());
    assert(parser.consume(sizeof(image), nullptr, 0, &data, &length));
    assert(!data && !length && !parser.complete());
    assert(!parser.consume(sizeof(image), nullptr, 1, &data, &length));
    assert(!parser.consume(sizeof(image), image, sizeof(image), &data, &length));

    parser = OtaElementParser{};
    assert(parser.consume(sizeof(image), image, 4, &data, &length));
    assert(!parser.consume(sizeof(image) + 1, image + 4, 6, &data, &length));

    parser = OtaElementParser{};
    assert(!parser.consume(sizeof(image) - 1, image, sizeof(image), &data, &length));

    uint8_t invalid[] = {1, 0, 4, 0, 0, 0};
    parser = OtaElementParser{};
    assert(!parser.consume(sizeof(image), invalid, sizeof(invalid), &data, &length));
    invalid[0] = 0;
    invalid[2] = 5;
    parser = OtaElementParser{};
    assert(!parser.consume(sizeof(image), invalid, sizeof(invalid), &data, &length));

    parser = OtaElementParser{};
    assert(!parser.consume(6, image, 6, &data, &length));
    parser = OtaElementParser{};
    assert(parser.consume(sizeof(image), image, sizeof(image), &data, &length));
    assert(parser.complete());
    assert(!parser.consume(sizeof(image), image, 1, &data, &length));
    assert(!parser.complete());

    parser = OtaElementParser{};
    assert(parser.consume(sizeof(image), image, sizeof(image), &data, &length));
    assert(parser.complete());
    puts("OTA parser tests passed (512 fragmentation patterns and invalid-input/reset cases)");
}
