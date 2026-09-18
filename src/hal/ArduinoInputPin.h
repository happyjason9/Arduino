#pragma once
#include <Arduino.h>
#include "IGpio.h"

// HAL 層：輸入腳位，啟用內部上拉電阻。
//
// 接線只要兩條，不需要外接電阻：
//   按鈕一腳 -> Arduino 這支腳位
//   按鈕另一腳 -> Arduino GND
// 沒按時內部上拉把腳位拉到 5V (HIGH)，按下時接到 GND (LOW)。
class ArduinoInputPin : public IGpio {
public:
    explicit ArduinoInputPin(uint8_t pin) : pin_(pin) {}
    void init() override            { pinMode(pin_, INPUT_PULLUP); }
    void write(bool) override       { /* 輸入腳位，寫入無意義 */ }
    bool read() const override      { return digitalRead(pin_) == HIGH; }
private:
    uint8_t pin_;
};
