#pragma once
#include <Arduino.h>
#include "IStepPulse.h"

// HAL 層：CNC Shield 上的 A4988 / DRV8825 步進驅動器。
//
// 跟 Dm542Driver 的差別是「正負邏輯」：DM542 走共陽極接法，訊號拉 LOW 才導通
// 光耦；A4988 是直接的 CMOS 輸入，STEP 拉 HIGH 才是有效緣。所以這裡不反相。
class A4988Driver : public IStepPulse {
public:
    A4988Driver(uint8_t stepPin, uint8_t dirPin, bool invertDir = false)
        : stp_(stepPin), dir_(dirPin), invertDir_(invertDir) {}

    void init() override {
        pinMode(stp_, OUTPUT);
        pinMode(dir_, OUTPUT);
        digitalWrite(stp_, LOW);
        digitalWrite(dir_, LOW);
    }

    void setDirection(bool forward) override {
        bool level = invertDir_ ? !forward : forward;
        digitalWrite(dir_, level ? HIGH : LOW);
    }

    void pulseHigh() override { digitalWrite(stp_, HIGH); }
    void pulseLow()  override { digitalWrite(stp_, LOW);  }

private:
    uint8_t stp_;
    uint8_t dir_;
    bool    invertDir_;   // 轉向反了改這個，不用動接線
};
