#pragma once
#include <stdint.h>
#include "Stepper.h"
#include "Button.h"

// 原點復歸：純邏輯的非阻塞狀態機。
//
// 標準四段流程 —— 為什麼要碰兩次：第一次快速接近是為了省時間，但高速碰到
// 開關時的觸發點會受慣性和掃描頻率影響，重現性差。退開後用慢速再碰一次，
// 觸發點才穩定，重複定位精度可以差到一個數量級。
//
// 開機時滑塊可能已經壓在開關上，所以第一步要先確認並退開 —— 否則
// 「朝開關移動直到觸發」會在原地立刻成立，根本沒有真的歸位。
class Homing {
public:
    enum class Result : uint8_t { Busy, Done, Failed };

    Homing(Stepper& motor, Button& limit,
           bool towardLimit,          // 哪個方向是朝原點
           uint32_t fastHalfUs,       // 快速接近的半週期
           uint32_t slowHalfUs,       // 慢速二次接近
           uint32_t backoffSteps,     // 退開步數
           uint32_t maxSteps)         // 逾時保護
        : motor_(motor), limit_(limit), toward_(towardLimit),
          fast_(fastHalfUs), slow_(slowHalfUs),
          backoff_(backoffSteps), maxSteps_(maxSteps) {}

    void start() {
        phase_ = Phase::Start;
        result_ = Result::Busy;
    }

    Result result() const { return result_; }
    bool   isBusy() const { return result_ == Result::Busy; }

    void tick() {
        if (result_ != Result::Busy) return;

        switch (phase_) {
        case Phase::Start:
            motor_.setHalfPeriodUs(fast_);
            if (limit_.isTriggered()) {
                // 本來就壓在開關上 —— 先退開，否則下一步會立刻「成立」
                motor_.moveSteps(backoff_, !toward_);
                phase_ = Phase::BackoffFirst;
            } else {
                motor_.moveUntilSignal(maxSteps_, toward_);
                phase_ = Phase::SeekFast;
            }
            return;

        case Phase::BackoffFirst:
            if (motor_.isMoving()) return;
            motor_.moveUntilSignal(maxSteps_, toward_);
            phase_ = Phase::SeekFast;
            return;

        case Phase::SeekFast:
            if (limit_.isTriggered()) {
                motor_.stop();
                motor_.moveSteps(backoff_, !toward_);
                phase_ = Phase::BackoffSecond;
                return;
            }
            if (!motor_.isMoving()) {       // 走完 maxSteps 都沒碰到
                result_ = Result::Failed;
            }
            return;

        case Phase::BackoffSecond:
            if (motor_.isMoving()) return;
            motor_.setHalfPeriodUs(slow_);  // 慢速再碰一次，觸發點才穩定
            motor_.moveUntilSignal(backoff_ * 4, toward_);
            phase_ = Phase::SeekSlow;
            return;

        case Phase::SeekSlow:
            if (limit_.isTriggered()) {
                motor_.stop();
                motor_.setHalfPeriodUs(fast_);   // 還原速度給後續動作用
                result_ = Result::Done;
                return;
            }
            if (!motor_.isMoving()) {       // 退開後回頭卻找不到 —— 開關有問題
                motor_.setHalfPeriodUs(fast_);
                result_ = Result::Failed;
            }
            return;
        }
    }

private:
    enum class Phase : uint8_t {
        Start, BackoffFirst, SeekFast, BackoffSecond, SeekSlow
    };

    Stepper& motor_;
    Button&  limit_;
    bool     toward_;
    uint32_t fast_, slow_, backoff_, maxSteps_;
    Phase    phase_  = Phase::Start;
    Result   result_ = Result::Done;
};
