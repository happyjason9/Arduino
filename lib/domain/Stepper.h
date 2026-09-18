#pragma once
#include <stdint.h>
#include "IStepPulse.h"

// 純邏輯：沒有 #include <Arduino.h>，沒有 delay()，不讀 micros()。
// 時間由外部傳入 (tick 的參數)，所以測試可以任意操控時間。
//
// 為什麼用 micros 而不是 millis：1600 pulse/rev 在 60 rpm 下，半週期只有
// 312us —— 毫秒的解析度根本畫不出脈波。
class Stepper {
public:
    // cruiseHalfPeriodUs: 巡航(全速)時的脈波半週期。越小越快。
    //   轉速(rpm) = 60 * 1e6 / (2 * halfPeriodUs * pulsePerRev)
    //   例：halfPeriodUs=50, pulsePerRev=1600 -> 375 rpm
    Stepper(IStepPulse& drv, uint32_t pulsePerRev, uint32_t cruiseHalfPeriodUs)
        : drv_(drv), ppr_(pulsePerRev),
          cruise_(cruiseHalfPeriodUs), halfPeriod_(cruiseHalfPeriodUs),
          start_(cruiseHalfPeriodUs) {}

    // 設定加速斜坡：起步用 startHalfPeriodUs (較慢)，經過 rampSteps 步之後
    // 線性收斂到巡航速度；接近終點時再對稱地減速回起步速度。
    //
    // 為什麼需要：步進馬達的轉子有慣量，從靜止瞬間跳到高速時磁場會跑在
    // 轉子前面，靠磁力硬拽 —— 空載勉強跟得上，一接負載就失步或堵轉。
    // 失步是靜默的：DM542 沒有編碼器回授，漏掉的步數沒人會知道。
    //
    // startHalfPeriodUs 必須 >= 巡航值 (起步一定比全速慢)；傳入較小值會被夾住。
    void setRamp(uint32_t startHalfPeriodUs, uint32_t rampSteps) {
        start_     = startHalfPeriodUs < cruise_ ? cruise_ : startHalfPeriodUs;
        rampSteps_ = rampSteps;
    }

    // 排入一段移動。remaining 會累加，所以連續呼叫是「再多走這麼多步」。
    void moveSteps(uint32_t steps, bool forward) {
        if (steps == 0) return;
        if (forward != forward_) {
            forward_ = forward;
            pendingDirChange_ = true;   // 方向變了才需要重新等 DIR setup time
        }
        remaining_ += steps;
        moveTotal_ = remaining_;   // 累加後重算，斜坡以「現在的剩餘」為一整段
        openEnded_ = false;
        hitLimit_  = false;
    }

    void moveRevolutions(uint32_t revs, bool forward) {
        moveSteps(revs * ppr_, forward);
    }

    // 持續移動直到外部喊停 (碰到限位開關、感測器觸發)。
    // maxSteps 是逾時保護：走完還沒被喊停就自己停下，isMoving() 轉 false，
    // 呼叫端用 stepsRemaining() == 0 判斷是「正常觸發」還是「跑到上限」。
    //
    // 用途：homing、推到底部這類「步數事先不知道」的動作。
    void moveUntilSignal(uint32_t maxSteps, bool forward) {
        moveSteps(maxSteps, forward);
        openEnded_ = true;
    }

    // 逾時了嗎 —— moveUntilSignal 走完 maxSteps 卻沒被 stop()。
    bool hitStepLimit() const { return hitLimit_; }

    // 立刻停止並丟棄剩餘步數。脈波收在閒置準位，不會卡在半個脈波中間。
    void stop() {
        openEnded_ = false;
        remaining_ = 0;
        moveTotal_ = 0;
        if (phase_ == Phase::PulseActive) {
            drv_.pulseLow();
        }
        phase_ = Phase::Idle;
    }

    // 非阻塞：每次呼叫只推進一個狀態，立刻返回。
    // 呼叫頻率必須高於脈波頻率，否則實際轉速會被 loop 的速度拖慢。
    void tick(uint32_t nowUs) {
        switch (phase_) {
        case Phase::Idle:
            if (remaining_ == 0) return;
            drv_.setDirection(forward_);
            if (pendingDirChange_) {
                pendingDirChange_ = false;
                phase_    = Phase::DirSettle;   // DM542 要求 DIR 領先 PUL >= 5us
                lastEdge_ = nowUs;
                return;
            }
            beginPulse(nowUs);
            return;

        case Phase::DirSettle:
            if (nowUs - lastEdge_ < kDirSetupUs) return;
            beginPulse(nowUs);
            return;

        case Phase::PulseActive:
            if (nowUs - lastEdge_ < halfPeriod_) return;
            drv_.pulseLow();
            lastEdge_ = nowUs;
            phase_    = Phase::PulseIdle;
            return;

        case Phase::PulseIdle:
            if (nowUs - lastEdge_ < halfPeriod_) return;
            // 一個完整脈波走完，這時才算真的走了一步。
            --remaining_;
            ++position_;
            phase_ = Phase::Idle;
            if (remaining_ == 0 && openEnded_) {
                hitLimit_  = true;    // 走完上限還沒被喊停 = 卡料/故障
                openEnded_ = false;
            }
            if (remaining_ > 0) {
                beginPulse(nowUs);   // 接著下一步，不用繞回 Idle 多等一輪
            }
            return;
        }
    }

    bool     isMoving()      const { return remaining_ > 0 || phase_ != Phase::Idle; }
    uint32_t stepsRemaining() const { return remaining_; }
    uint32_t stepsTaken()     const { return position_; }
    bool     direction()      const { return forward_; }

    // 中途變速是允許的，會從下一個邊緣開始生效。
    // 有斜坡時這改的是巡航速度，斜坡起點不動。
    void setHalfPeriodUs(uint32_t us) {
        cruise_ = us;
        if (start_ < cruise_) start_ = cruise_;
    }

    // 目前這一步實際使用的半週期 (斜坡進行中會逐步變化)。
    uint32_t currentHalfPeriodUs() const { return halfPeriod_; }

private:
    enum class Phase : uint8_t { Idle, DirSettle, PulseActive, PulseIdle };

    void beginPulse(uint32_t nowUs) {
        halfPeriod_ = rampedHalfPeriod();
        drv_.pulseHigh();
        lastEdge_ = nowUs;
        phase_    = Phase::PulseActive;
    }

    // 依「這一步在整段行程中的位置」算出該用的半週期。
    // 起步和收尾各佔 rampSteps_ 步，中間巡航；行程太短時兩端各分一半，
    // 所以永遠不會加速到一半就被迫急停。
    uint32_t rampedHalfPeriod() const {
        if (rampSteps_ == 0 || start_ == cruise_) return cruise_;

        // open-ended 移動的終點是未知的 (等訊號才停)，所以只做加速段，
        // 不做減速 —— 否則會依 maxSteps 在完全錯誤的位置開始減速。
        if (openEnded_) {
            uint32_t done = moveTotal_ - remaining_;
            if (done >= rampSteps_) return cruise_;
            return start_ - ((start_ - cruise_) * done) / rampSteps_;
        }

        uint32_t doneInMove = moveTotal_ - remaining_;   // 已走幾步
        uint32_t ramp = rampSteps_;
        if (moveTotal_ < ramp * 2) ramp = moveTotal_ / 2;  // 短行程：兩端對半分
        if (ramp == 0) return cruise_;

        // 距離「最近的一端」有幾步 —— 兩端都是慢speed，中間快。
        uint32_t fromEdge = doneInMove < remaining_ ? doneInMove : remaining_ - 1;
        if (fromEdge >= ramp) return cruise_;            // 已在巡航區

        // 線性內插：fromEdge=0 -> start_，fromEdge=ramp -> cruise_
        uint32_t span = start_ - cruise_;                // setRamp 保證 start_ >= cruise_
        return start_ - (span * fromEdge) / ramp;
    }

    static const uint32_t kDirSetupUs = 10;   // DM542 規格要求 >= 5us，取兩倍當餘裕

    IStepPulse& drv_;
    uint32_t    ppr_;
    uint32_t    cruise_;        // 全速時的半週期
    uint32_t    halfPeriod_;    // 這一步實際用的半週期
    uint32_t    start_;         // 斜坡起點的半週期
    uint32_t    rampSteps_ = 0; // 0 = 不用斜坡，定速
    uint32_t    remaining_ = 0;
    uint32_t    moveTotal_ = 0;   // 這一段行程的總步數，斜坡用來定位兩端
    uint32_t    position_  = 0;
    uint32_t    lastEdge_  = 0;
    Phase       phase_     = Phase::Idle;
    bool        forward_   = true;
    bool        pendingDirChange_ = true;   // 第一次動作也要等 DIR 穩定
    bool        openEnded_ = false;         // moveUntilSignal 排的，等外部喊停
    bool        hitLimit_  = false;         // open-ended 走完上限仍未被喊停
};
