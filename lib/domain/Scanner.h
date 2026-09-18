#pragma once
#include <stdint.h>
#include "Stepper.h"
#include "ICapture.h"

// 相機 XY 蛇形掃描：純邏輯的非阻塞狀態機。
//
// 蛇形 (boustrophedon) 走位：偶數列往 +X、奇數列往 -X。比每列都回到起點
// 省掉一次橫越全幅的空跑，時間差在列數多時很可觀。
//
// 掃完依步數回到原點 —— 不重新 homing。呼叫端可以用原點開關驗證位置對不對，
// 對不上就代表這輪丟步了 (見 system.ino 原本的 camVerifyAtOrigin 想法)。
class Scanner {
public:
    enum class Result : uint8_t { Busy, Done, CaptureFailed, LimitHit };

    // yScanDir：換列時 Y 要往哪個方向走。掃描是從拍照起點往「遠離原點」的方向
    // 推進，所以這裡要傳 !CAM_Y_TOWARD_HOME —— 傳錯的話會朝原點側掃回去。
    Scanner(Stepper& x, Stepper& y, ICapture& cam,
            uint16_t cols, uint16_t rows,
            uint32_t stepsPerCol, uint32_t stepsPerRow,
            bool yScanDir = true)
        : x_(x), y_(y), cam_(cam),
          cols_(cols), rows_(rows),
          stepsPerCol_(stepsPerCol), stepsPerRow_(stepsPerRow),
          yScanDir_(yScanDir) {}

    void start() {
        col_ = 0; row_ = 0;
        phase_  = Phase::Capture;
        result_ = Result::Busy;
        started_ = false;
    }

    Result result() const { return result_; }
    bool   isBusy() const { return result_ == Result::Busy; }
    uint16_t currentCol() const { return col_; }
    uint16_t currentRow() const { return row_; }

    // 限位被撞到時由呼叫端通知 —— 掃描過程中不該碰到任何限位。
    void abortOnLimit() {
        if (result_ != Result::Busy) return;
        x_.stop(); y_.stop();
        result_ = Result::LimitHit;
    }

    void tick() {
        if (result_ != Result::Busy) return;

        switch (phase_) {
        case Phase::Capture:
            if (!started_) { cam_.request(col_, row_); started_ = true; return; }
            if (cam_.isFailed()) { result_ = Result::CaptureFailed; return; }
            if (!cam_.isDone()) return;
            started_ = false;
            advance();
            return;

        case Phase::MoveX:
            if (x_.isMoving()) return;
            phase_ = Phase::Capture;
            return;

        case Phase::MoveY:
            if (y_.isMoving()) return;
            phase_ = Phase::Capture;
            return;

        case Phase::ReturnX:
        case Phase::ReturnY:
            // 回程 X/Y 一起走 —— 兩軸互相獨立，沒有理由排隊。
            if (x_.isMoving() || y_.isMoving()) return;
            result_ = Result::Done;
            return;
        }
    }

    // 最後一格拍完、回程已經排下去了嗎？呼叫端可以據此提早讓輸送台動起來，
    // 不必等相機真的走回定位。
    bool isReturning() const {
        return result_ == Result::Busy &&
               (phase_ == Phase::ReturnX || phase_ == Phase::ReturnY);
    }

private:
    enum class Phase : uint8_t { Capture, MoveX, MoveY, ReturnX, ReturnY };

    // 拍完一點之後決定下一步：同列往旁邊、換列往下、或全部拍完回原點。
    void advance() {
        bool forwardRow = (row_ % 2 == 0);      // 偶數列往 +X

        if (col_ + 1 < cols_) {
            ++col_;
            x_.moveSteps(stepsPerCol_, forwardRow);
            phase_ = Phase::MoveX;
            return;
        }
        if (row_ + 1 < rows_) {
            ++row_;
            col_ = 0;                            // 邏輯上回到第 0 欄
            y_.moveSteps(stepsPerRow_, yScanDir_);
            phase_ = Phase::MoveY;
            return;
        }
        beginReturn();
    }

    // 掃完了，依步數走回原點。停在哪一端要看最後一列的走向。
    void beginReturn() {
        bool moving = false;
        // 最後一列若是往 +X 掃的，相機停在 +X 端，要整排走回來。
        if (((rows_ - 1) % 2 == 0) && cols_ > 1) {
            x_.moveSteps(stepsPerCol_ * (uint32_t)(cols_ - 1), false);
            moving = true;
        }
        if (rows_ > 1) {
            y_.moveSteps(stepsPerRow_ * (uint32_t)(rows_ - 1), !yScanDir_);
            moving = true;
        }
        phase_  = Phase::ReturnX;          // ReturnX/ReturnY 現在等效
        if (!moving) result_ = Result::Done;
    }

    bool      yScanDir_ = true;

    Stepper&  x_;
    Stepper&  y_;
    ICapture& cam_;
    uint16_t  cols_, rows_;
    uint32_t  stepsPerCol_, stepsPerRow_;
    uint16_t  col_ = 0, row_ = 0;
    Phase     phase_   = Phase::Capture;
    Result    result_  = Result::Done;
    bool      started_ = false;
};
