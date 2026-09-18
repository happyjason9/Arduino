#pragma once
#include <stdint.h>
#include "IGpio.h"

// 純邏輯：沒有 #include <Arduino.h>，沒有 delay()，不讀 millis()。
// 時間由外部傳入 (tick 的參數)，所以測試可以任意操控時間。
class Blinker {
public:
    Blinker(IGpio& led, uint32_t periodMs)
        : led_(led), period_(periodMs) {}

    // 非阻塞：每次呼叫只判斷「時間到了沒」，立刻返回。
    void tick(uint32_t nowMs) {
        if (nowMs - lastToggle_ >= period_) {   // 無號數相減，millis() 溢位也正確
            lastToggle_ = nowMs;
            on_ = !on_;
            led_.write(on_);
        }
    }

    bool isOn() const { return on_; }

    // 中途改變閃爍速度，用來表示狀態 (例如正常慢閃、故障快閃)。
    void setPeriod(uint32_t periodMs) { period_ = periodMs; }
    uint32_t period() const           { return period_; }

private:
    IGpio&   led_;
    uint32_t period_;
    uint32_t lastToggle_ = 0;
    bool     on_ = false;
};
