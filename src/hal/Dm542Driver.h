#pragma once
#include <Arduino.h>
#include "IStepPulse.h"

// HAL 層：整個專案唯一碰硬體 API 的地方。
//
// 共陽極 (common anode) 接法：
//   Arduino 5V -> DM542 PUL+ / DIR+ (並接)
//   Arduino D3 -> DM542 PUL-
//   Arduino D2 -> DM542 DIR-
// 電流從 5V 經光耦流回 Arduino 腳位，所以腳位拉 LOW 才是「訊號有效」。
// 邏輯上的 true 在這裡被反相成電氣上的 LOW —— domain 層不必知道這件事。
class Dm542Driver : public IStepPulse {
public:
    Dm542Driver(uint8_t pulPin, uint8_t dirPin, bool invertDir = false)
        : pul_(pulPin), dir_(dirPin), invertDir_(invertDir) {}

    void init() override {
        pinMode(pul_, OUTPUT);
        pinMode(dir_, OUTPUT);
        digitalWrite(pul_, HIGH);   // HIGH = 光耦不導通 = 閒置
        digitalWrite(dir_, HIGH);
    }

    void setDirection(bool forward) override {
        bool level = invertDir_ ? forward : !forward;
        digitalWrite(dir_, level ? HIGH : LOW);
    }

    void pulseHigh() override { digitalWrite(pul_, LOW);  }   // 有效緣 = 拉低
    void pulseLow()  override { digitalWrite(pul_, HIGH); }

private:
    uint8_t pul_;
    uint8_t dir_;
    bool    invertDir_;   // 轉向反了就把這個設 true，不用動接線
};
