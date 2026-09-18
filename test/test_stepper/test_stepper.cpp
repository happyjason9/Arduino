#include <unity.h>
#include "IStepPulse.h"
#include "Stepper.h"

// 假的驅動器：不碰硬體，只記錄被呼叫的狀況。
class FakePulse : public IStepPulse {
public:
    void init() override                    { initCalled = true; }
    void setDirection(bool forward) override { dir = forward; ++dirCount; }
    void pulseHigh() override               { active = true;  ++highCount; }
    void pulseLow()  override               { active = false; ++lowCount; }

    bool initCalled = false;
    bool dir        = false;
    bool active     = false;
    int  dirCount   = 0;
    int  highCount  = 0;
    int  lowCount   = 0;
};

// 把時間推進到走完 n 個完整脈波為止，回傳最後的時間戳。
// 每 10us 呼叫一次 tick，模擬 loop 跑得比脈波快。
static uint32_t runUntilIdle(Stepper& s, uint32_t startUs, uint32_t maxUs = 200000) {
    uint32_t t = startUs;
    while (s.isMoving() && t - startUs < maxUs) {
        s.tick(t);
        t += 10;
    }
    return t;
}

void test_starts_idle(void) {
    FakePulse drv;
    Stepper   s(drv, 1600, 300);
    TEST_ASSERT_FALSE(s.isMoving());
    TEST_ASSERT_EQUAL_UINT32(0, s.stepsRemaining());
}

void test_move_zero_steps_does_nothing(void) {
    FakePulse drv;
    Stepper   s(drv, 1600, 300);
    s.moveSteps(0, true);
    TEST_ASSERT_FALSE(s.isMoving());
}

void test_waits_for_dir_setup_before_first_pulse(void) {
    FakePulse drv;
    Stepper   s(drv, 1600, 300);
    s.moveSteps(1, true);
    s.tick(0);                    // 進入 DirSettle，DIR 已設定但還不能發脈波
    TEST_ASSERT_EQUAL_INT(1, drv.dirCount);
    TEST_ASSERT_EQUAL_INT(0, drv.highCount);
    s.tick(9);                    // 還沒滿 10us
    TEST_ASSERT_EQUAL_INT(0, drv.highCount);
    s.tick(10);                   // 到了才發
    TEST_ASSERT_EQUAL_INT(1, drv.highCount);
}

void test_pulse_has_symmetric_half_periods(void) {
    FakePulse drv;
    Stepper   s(drv, 1600, 300);
    s.moveSteps(1, true);
    s.tick(0);
    s.tick(10);                   // 脈波拉高
    TEST_ASSERT_TRUE(drv.active);
    s.tick(10 + 299);             // 半週期還沒到
    TEST_ASSERT_TRUE(drv.active);
    s.tick(10 + 300);             // 拉低
    TEST_ASSERT_FALSE(drv.active);
    TEST_ASSERT_EQUAL_UINT32(1, s.stepsRemaining());   // 還沒算走完
    s.tick(10 + 600);             // 後半週期結束，這步才算數
    TEST_ASSERT_EQUAL_UINT32(0, s.stepsRemaining());
    TEST_ASSERT_EQUAL_UINT32(1, s.stepsTaken());
}

void test_emits_exact_pulse_count(void) {
    FakePulse drv;
    Stepper   s(drv, 1600, 300);
    s.moveSteps(50, true);
    runUntilIdle(s, 0);
    TEST_ASSERT_FALSE(s.isMoving());
    TEST_ASSERT_EQUAL_INT(50, drv.highCount);
    TEST_ASSERT_EQUAL_INT(50, drv.lowCount);
    TEST_ASSERT_EQUAL_UINT32(50, s.stepsTaken());
}

void test_move_revolutions_uses_pulse_per_rev(void) {
    FakePulse drv;
    Stepper   s(drv, 200, 10);     // 小一點的 ppr 讓測試跑得快
    s.moveRevolutions(2, true);
    TEST_ASSERT_EQUAL_UINT32(400, s.stepsRemaining());
}

void test_direction_change_re_arms_dir_setup(void) {
    FakePulse drv;
    Stepper   s(drv, 200, 10);
    s.moveSteps(5, true);
    uint32_t t = runUntilIdle(s, 0);
    int dirCountAfterForward = drv.dirCount;

    s.moveSteps(5, false);         // 換方向
    runUntilIdle(s, t);
    TEST_ASSERT_FALSE(drv.dir);
    TEST_ASSERT_TRUE(drv.dirCount > dirCountAfterForward);
    TEST_ASSERT_EQUAL_INT(10, drv.highCount);
}

void test_same_direction_does_not_re_settle(void) {
    FakePulse drv;
    Stepper   s(drv, 200, 10);
    s.moveSteps(3, true);
    uint32_t t = runUntilIdle(s, 0);
    s.moveSteps(3, true);          // 同方向，不該再等 DIR setup
    runUntilIdle(s, t);
    TEST_ASSERT_EQUAL_INT(6, drv.highCount);
}

void test_stop_discards_remaining_and_parks_low(void) {
    FakePulse drv;
    Stepper   s(drv, 1600, 300);
    s.moveSteps(100, true);
    s.tick(0);
    s.tick(10);                    // 停在脈波高準位
    TEST_ASSERT_TRUE(drv.active);
    s.stop();
    TEST_ASSERT_FALSE(drv.active); // 不能卡在半個脈波中間
    TEST_ASSERT_FALSE(s.isMoving());
    TEST_ASSERT_EQUAL_UINT32(0, s.stepsRemaining());
}

// micros() 在約 71.6 分鐘後會溢位歸零 —— 比 millis() 更快遇到。
// 無號數相減讓這個 case 依然正確，這裡把它釘住。
void test_survives_micros_overflow(void) {
    FakePulse drv;
    Stepper   s(drv, 1600, 300);
    uint32_t nearMax = 0xFFFFFF00;
    s.moveSteps(3, true);
    runUntilIdle(s, nearMax);      // 過程中會跨過 0xFFFFFFFF
    TEST_ASSERT_FALSE(s.isMoving());
    TEST_ASSERT_EQUAL_INT(3, drv.highCount);
}


// --- 加速斜坡 ---------------------------------------------------------

// 走完一整段，記錄每一步實際用的半週期。
// out[i] = 第 i 步 (0-based) 使用的半週期。
static void collectHalfPeriods(Stepper& s, uint32_t out[], uint32_t maxOut) {
    uint32_t t = 0;
    while (s.isMoving() && t < 5000000) {
        // stepsTaken() 是「已完成」的步數，所以現在進行中的是第 stepsTaken() 步。
        uint32_t idx = s.stepsTaken();
        if (idx < maxOut) out[idx] = s.currentHalfPeriodUs();
        s.tick(t);
        t += 5;
    }
}

void test_no_ramp_by_default_is_constant_speed(void) {
    FakePulse drv;
    Stepper   s(drv, 200, 100);
    s.moveSteps(10, true);
    s.tick(0); s.tick(10);
    TEST_ASSERT_EQUAL_UINT32(100, s.currentHalfPeriodUs());
    runUntilIdle(s, 0);
    TEST_ASSERT_EQUAL_INT(10, drv.highCount);
}

void test_ramp_starts_slow(void) {
    FakePulse drv;
    Stepper   s(drv, 200, 50);
    s.setRamp(500, 10);          // 起步 500us，10 步收斂到 50us
    s.moveSteps(100, true);
    s.tick(0); s.tick(10);       // 第一步
    TEST_ASSERT_EQUAL_UINT32(500, s.currentHalfPeriodUs());
}

void test_ramp_reaches_cruise_in_middle(void) {
    FakePulse drv;
    Stepper   s(drv, 200, 50);
    s.setRamp(500, 10);
    s.moveSteps(100, true);
    uint32_t hp[100] = {0};
    collectHalfPeriods(s, hp, 100);
    // 中段應該已經是巡航速度
    TEST_ASSERT_EQUAL_UINT32(50, hp[50]);
}

void test_ramp_is_monotonic_speeding_up(void) {
    FakePulse drv;
    Stepper   s(drv, 200, 50);
    s.setRamp(500, 10);
    s.moveSteps(100, true);
    uint32_t hp[100] = {0};
    collectHalfPeriods(s, hp, 100);
    // 前 10 步的半週期只能越來越小 (越來越快)，不能反覆
    for (int i = 1; i < 10; ++i) {
        TEST_ASSERT_TRUE(hp[i] <= hp[i - 1]);
    }
}

void test_ramp_slows_down_at_the_end(void) {
    FakePulse drv;
    Stepper   s(drv, 200, 50);
    s.setRamp(500, 10);
    s.moveSteps(100, true);
    uint32_t t = 0;
    uint32_t lastHp = 0;
    // 跑到剩最後一步，看速度有沒有降回起步值附近
    while (s.isMoving() && t < 5000000) {
        s.tick(t);
        if (s.stepsRemaining() == 1) { lastHp = s.currentHalfPeriodUs(); break; }
        t += 5;
    }
    TEST_ASSERT_EQUAL_UINT32(500, lastHp);
}

void test_ramp_still_emits_exact_step_count(void) {
    FakePulse drv;
    Stepper   s(drv, 200, 50);
    s.setRamp(500, 10);
    s.moveSteps(100, true);
    runUntilIdle(s, 0, 5000000);
    TEST_ASSERT_FALSE(s.isMoving());
    TEST_ASSERT_EQUAL_INT(100, drv.highCount);   // 斜坡不能吃掉或多發脈波
    TEST_ASSERT_EQUAL_UINT32(100, s.stepsTaken());
}

// 行程比斜坡還短時，不能加速到一半就急停 —— 兩端各分一半。
void test_short_move_splits_ramp(void) {
    FakePulse drv;
    Stepper   s(drv, 200, 50);
    s.setRamp(500, 50);          // 斜坡 50 步，但只走 6 步
    s.moveSteps(6, true);
    runUntilIdle(s, 0, 5000000);
    TEST_ASSERT_EQUAL_INT(6, drv.highCount);
}

void test_ramp_start_cannot_be_faster_than_cruise(void) {
    FakePulse drv;
    Stepper   s(drv, 200, 100);
    s.setRamp(50, 10);           // 起步比巡航還快 —— 不合理，應被夾住
    s.moveSteps(20, true);
    s.tick(0); s.tick(10);
    TEST_ASSERT_EQUAL_UINT32(100, s.currentHalfPeriodUs());
}


// --- moveUntilSignal (步數事先未知的移動) --------------------

void test_move_until_signal_runs_until_stopped(void) {
    FakePulse drv;
    Stepper   s(drv, 200, 10);
    s.moveUntilSignal(100000, true);
    uint32_t t = 0;
    // 跑 30 步後模擬限位觸發
    while (s.isMoving() && s.stepsTaken() < 30 && t < 500000) { s.tick(t); t += 5; }
    uint32_t taken = s.stepsTaken();
    s.stop();
    TEST_ASSERT_FALSE(s.isMoving());
    TEST_ASSERT_EQUAL_UINT32(30, taken);
    // 最後一個脈波可能已經開始但還沒走完，所以 highCount 會多一個。
    TEST_ASSERT_TRUE(drv.highCount >= 30 && drv.highCount <= 31);
    TEST_ASSERT_FALSE(s.hitStepLimit());   // 是被喊停的，不是逾時
}

void test_move_until_signal_times_out_at_max_steps(void) {
    FakePulse drv;
    Stepper   s(drv, 200, 10);
    s.moveUntilSignal(20, true);      // 沒人喊停
    runUntilIdle(s, 0, 5000000);
    TEST_ASSERT_FALSE(s.isMoving());
    TEST_ASSERT_EQUAL_INT(20, drv.highCount);
    TEST_ASSERT_TRUE(s.hitStepLimit());    // 跑到上限 = 卡料/故障
}

void test_normal_move_never_reports_step_limit(void) {
    FakePulse drv;
    Stepper   s(drv, 200, 10);
    s.moveSteps(20, true);
    runUntilIdle(s, 0, 5000000);
    TEST_ASSERT_FALSE(s.hitStepLimit());   // 一般移動走完不算逾時
}

void test_move_steps_clears_previous_limit_flag(void) {
    FakePulse drv;
    Stepper   s(drv, 200, 10);
    s.moveUntilSignal(5, true);
    runUntilIdle(s, 0, 5000000);
    TEST_ASSERT_TRUE(s.hitStepLimit());
    s.moveSteps(5, true);                  // 新的移動要清掉舊旗標
    TEST_ASSERT_FALSE(s.hitStepLimit());
}

// open-ended 移動的終點未知，只能加速，不能依 maxSteps 減速。
void test_open_ended_move_accelerates_but_never_decelerates(void) {
    FakePulse drv;
    Stepper   s(drv, 200, 50);
    s.setRamp(500, 10);
    s.moveUntilSignal(100, true);
    uint32_t hp[100] = {0};
    collectHalfPeriods(s, hp, 100);
    TEST_ASSERT_EQUAL_UINT32(500, hp[0]);    // 起步慢
    TEST_ASSERT_EQUAL_UINT32(50, hp[50]);    // 中段已巡航
    TEST_ASSERT_EQUAL_UINT32(50, hp[95]);    // 接近 maxSteps 也不該減速
}

int main(int, char**) {
    UNITY_BEGIN();
    RUN_TEST(test_starts_idle);
    RUN_TEST(test_move_zero_steps_does_nothing);
    RUN_TEST(test_waits_for_dir_setup_before_first_pulse);
    RUN_TEST(test_pulse_has_symmetric_half_periods);
    RUN_TEST(test_emits_exact_pulse_count);
    RUN_TEST(test_move_revolutions_uses_pulse_per_rev);
    RUN_TEST(test_direction_change_re_arms_dir_setup);
    RUN_TEST(test_same_direction_does_not_re_settle);
    RUN_TEST(test_stop_discards_remaining_and_parks_low);
    RUN_TEST(test_survives_micros_overflow);
    RUN_TEST(test_no_ramp_by_default_is_constant_speed);
    RUN_TEST(test_ramp_starts_slow);
    RUN_TEST(test_ramp_reaches_cruise_in_middle);
    RUN_TEST(test_ramp_is_monotonic_speeding_up);
    RUN_TEST(test_ramp_slows_down_at_the_end);
    RUN_TEST(test_ramp_still_emits_exact_step_count);
    RUN_TEST(test_short_move_splits_ramp);
    RUN_TEST(test_ramp_start_cannot_be_faster_than_cruise);
    RUN_TEST(test_move_until_signal_runs_until_stopped);
    RUN_TEST(test_move_until_signal_times_out_at_max_steps);
    RUN_TEST(test_normal_move_never_reports_step_limit);
    RUN_TEST(test_move_steps_clears_previous_limit_flag);
    RUN_TEST(test_open_ended_move_accelerates_but_never_decelerates);
    return UNITY_END();
}
