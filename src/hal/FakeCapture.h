#pragma once
#include <Arduino.h>
#include "ICapture.h"

// HAL 層：假裝 PC 端有回應的拍照介面 —— Python 還沒接上時用來跑完整流程。
//
// 行為跟 SerialCapture 一樣會印出 CAPTURE:<row>,<col>，方便對照序列埠輸出，
// 差別是不等 PC 回 OK，而是自己延遲 delayMs 之後就宣告完成。
// 延遲是為了模擬真實拍照的耗時，順便讓你看得清楚掃描走到哪一格。
//
// 非阻塞：request() 立刻返回，靠 poll() 推進 —— 與 SerialCapture 同介面，
// 所以 main 只要換掉宣告那一行就能切換，Scanner 完全不用動。
class FakeCapture : public ICapture {
public:
    explicit FakeCapture(uint32_t delayMs = 300) : delay_(delayMs) {}

    void request(uint16_t col, uint16_t row) override {
        Serial.print(F("CAPTURE:"));
        Serial.print(row);
        Serial.print(',');
        Serial.print(col);
        Serial.println(F("   [模擬：不等 PC 回應]"));
        sentAt_ = millis();
        done_   = false;
    }

    bool isDone()   const override { return done_; }
    bool isFailed() const override { return false; }   // 假的，永遠不會失敗

    void poll(uint32_t nowMs) {
        if (done_) return;
        if (nowMs - sentAt_ >= delay_) done_ = true;
    }

private:
    uint32_t delay_;
    uint32_t sentAt_ = 0;
    bool     done_   = false;
};
