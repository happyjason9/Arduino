#pragma once
#include <stdint.h>

// 步進驅動器的脈波介面 (DM542 的 PUL/DIR 兩條線)。
// domain 層只認識這個抽象，不知道底下是 DM542、是 mock、還是別的驅動器。
//
// 注意「邏輯準位」與「電氣準位」的分別：這裡的 true 一律代表「訊號有效」，
// 共陽極接法要拉 LOW 才導通光耦 —— 那是 HAL 的事，domain 不該知道。
class IStepPulse {
public:
    virtual ~IStepPulse() = default;
    virtual void init() = 0;
    virtual void setDirection(bool forward) = 0;  // true = 正轉
    virtual void pulseHigh() = 0;                 // 進入脈波的有效緣
    virtual void pulseLow()  = 0;                 // 回到閒置準位
};
