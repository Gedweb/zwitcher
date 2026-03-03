#ifndef ZWITCHER_DTO_H
#define ZWITCHER_DTO_H

#include <stdint.h>

class Relay {
private:
    unsigned int m_iPin;
    bool m_bState;

public:
    Relay(unsigned int pin);
    unsigned int getPin();
    bool getState();
    void setState(bool state);
};

class RelayEvent {
private:
    uint8_t m_endpoint; // 1-4
    bool m_state;

public:
    RelayEvent();
    RelayEvent(uint8_t endpoint, bool state);
    uint8_t getEndpoint();
    bool getState();
};

#endif //ZWITCHER_DTO_H
