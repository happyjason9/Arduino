#pragma once
#include <Arduino.h>
#include "IGpio.h"

// HAL 層：整個專案唯一碰硬體 API 的地方。
class ArduinoGpio : public IGpio {
public:
    explicit ArduinoGpio(uint8_t pin) : pin_(pin) {}
    void init() override            { pinMode(pin_, OUTPUT); }
    void write(bool high) override  { digitalWrite(pin_, high ? HIGH : LOW); }
    bool read() const override      { return digitalRead(pin_) == HIGH; }
private:
    uint8_t pin_;
};
