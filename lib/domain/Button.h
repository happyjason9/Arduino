#pragma once
#include <stdint.h>
#include "IGpio.h"

// 純邏輯：去彈跳 + 邊緣偵測。時間由外部傳入。
//
// 機械接點在切換的瞬間會在幾毫秒內快速通斷好幾次 (bounce) —— 瞬時按鈕和
// 自鎖式開關都一樣。Arduino 跑得夠快，會把一次動作讀成十幾次。
// 做法：狀態改變後要維持穩定超過 debounceMs 才採信。
//
// isDown()     = 目前是否導通 (自鎖式開關用這個讀狀態)
// wasPressed() = 這一次 tick 剛切到導通 (瞬時按鈕用這個做觸發)
//
// 接線用 INPUT_PULLUP，接到 GND 時讀到 LOW。activeLow 決定哪個電氣準位
// 算「有效」，對外一律用「有效 = true」的語意：
//
//   activeLow = true  (預設)：接點閉合接到 GND 才有效 —— 一般按鈕、NO 開關
//   activeLow = false        ：接點斷開才有效 —— NC 極限開關
//
// 為什麼極限開關要用 NC：接點平時閉合(讀 LOW)，壓下時斷開(讀 HIGH)。
// 這樣線鬆脫或開關損壞時同樣讀到 HIGH，會被判定成「觸發」而讓機器停下來 ——
// 失效安全 (fail-safe)。用 NO 的話線斷了會被讀成「一切正常」，
// 滑塊會繼續往死裡撞而且沒人發現。
class Button {
public:
    Button(IGpio& pin, uint32_t debounceMs = 30, bool activeLow = true)
        : pin_(pin), debounce_(debounceMs), activeLow_(activeLow) {}

    void tick(uint32_t nowMs) {
        pressedEdge_ = false;
        bool raw = activeLow_ ? !pin_.read() : pin_.read();

        if (raw != lastRaw_) {            // 準位在動，重新計時
            lastRaw_    = raw;
            lastChange_ = nowMs;
            return;
        }
        if (nowMs - lastChange_ < debounce_) return;   // 還沒穩定，先不採信

        if (raw != stable_) {
            stable_ = raw;
            if (stable_) pressedEdge_ = true;          // 只認「按下」那一刻
        }
    }

    // 這一次 tick 是否剛變成「有效」。回到無效不會觸發。
    bool wasPressed() const { return pressedEdge_; }
    bool isDown()     const { return stable_; }

    // 極限開關用的別名，語意比較直觀。
    bool isTriggered()  const { return stable_; }
    bool justTriggered() const { return pressedEdge_; }

private:
    IGpio&   pin_;
    uint32_t debounce_;
    bool     activeLow_;
    uint32_t lastChange_  = 0;
    bool     lastRaw_     = false;
    bool     stable_      = false;
    bool     pressedEdge_ = false;
};
