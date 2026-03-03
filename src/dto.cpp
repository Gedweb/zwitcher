#include "dto.h"

Relay::Relay(unsigned int pin) {
    m_iPin = pin;
    m_bState = false;
}

unsigned int Relay::getPin() {
    return m_iPin;
}

bool Relay::getState() {
    return m_bState;
}

void Relay::setState(bool state) {
    m_bState = state;
}

RelayEvent::RelayEvent() {
    m_endpoint = 0;
    m_state = false;
}

RelayEvent::RelayEvent(uint8_t endpoint, bool state) {
    m_endpoint = endpoint;
    m_state = state;
}

uint8_t RelayEvent::getEndpoint() {
    return m_endpoint;
}

bool RelayEvent::getState() {
    return m_state;
}
