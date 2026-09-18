#include <unity.h>
#include "IGpio.h"
#include "IStepPulse.h"
#include "Stepper.h"
#include "Button.h"
#include "Homing.h"

class FakePulse : public IStepPulse {
public:
    void init() override                     {}
    void setDirection(bool forward) override { dir = forward; }
    void pulseHigh() override                { ++highCount; }
    void pulseLow()  override                {}
    bool dir = false;
    int  highCount = 0;
};

class FakePin : public IGpio {
public:
    void init() override       {}
    void write(bool) override  {}
    bool read() const override { return level; }
    bool level = false;        // NC 極限開關：LOW = 正常
};

struct Rig {
    FakePulse drv;
    FakePin   pin;
    Stepper   motor{drv, 200, 20};
    Button    limit{pin, 0, false};     // debounce 0，測試裡好控制
    Homing    homing{motor, limit, true, 20, 60, 10, 1000};
    uint32_t  t = 0;

    void step(int n = 1) {
        for (int i = 0; i < n; ++i) {
            limit.tick(t / 1000);
            homing.tick();
            motor.tick(t);
            t += 5;
        }
    }
    // 跑到 homing 有結果，或超過上限
    void run(uint32_t maxIters = 400000) {
        uint32_t i = 0;
        while (homing.isBusy() && i++ < maxIters) step();
    }
    void trigger()   { pin.level = true;  }
    void untrigger() { pin.level = false; }
};

void test_homing_starts_busy(void) {
    Rig r;
    r.homing.start();
    TEST_ASSERT_TRUE(r.homing.isBusy());
}

// 一般情況：滑塊不在開關上，往原點走，碰到就停。
void test_homing_seeks_then_backs_off_then_reseeks(void) {
    Rig r;
    r.homing.start();
    r.step(200);                       // 讓它往原點跑一段
    TEST_ASSERT_TRUE(r.homing.isBusy());
    TEST_ASSERT_TRUE(r.drv.highCount > 0);

    r.trigger();                       // 快速接近碰到開關
    r.step(50);
    r.untrigger();                     // 退開的過程中開關鬆開
    r.step(400);
    r.trigger();                       // 慢速二次接近再碰到
    r.run();
    TEST_ASSERT_EQUAL_INT((int)Homing::Result::Done, (int)r.homing.result());
}

// 開機時滑塊已經壓在開關上 —— 必須先退開，不能當作已經歸位。
void test_homing_backs_off_first_if_already_on_switch(void) {
    Rig r;
    r.trigger();                       // 一開始就壓著
    r.step(2);                         // 讓 Button 讀到
    r.homing.start();
    r.step(5);
    // 應該朝「離開原點」方向退，而不是立刻宣告完成
    TEST_ASSERT_TRUE(r.homing.isBusy());
    TEST_ASSERT_FALSE(r.drv.dir);      // toward_=true，退開就是 false
}

// 走完 maxSteps 還沒碰到開關 = 開關壞了或線斷了。
void test_homing_fails_when_switch_never_triggers(void) {
    Rig r;
    r.homing.start();
    r.run();
    TEST_ASSERT_EQUAL_INT((int)Homing::Result::Failed, (int)r.homing.result());
}

// 慢速二次接近時找不到開關 —— 退開太多或開關接觸不良。
void test_homing_fails_if_slow_seek_misses(void) {
    Rig r;
    r.homing.start();
    r.step(200);
    r.trigger();
    r.step(50);
    r.untrigger();                     // 退開後就再也不觸發了
    r.run();
    TEST_ASSERT_EQUAL_INT((int)Homing::Result::Failed, (int)r.homing.result());
}

// 慢速接近要真的比較慢 —— 這是二次接近能提高精度的原因。
void test_homing_uses_slow_speed_for_second_approach(void) {
    Rig r;
    r.homing.start();
    r.step(200);
    TEST_ASSERT_EQUAL_UINT32(20, r.motor.currentHalfPeriodUs());   // 快速
    r.trigger();
    r.step(50);
    r.untrigger();
    r.step(400);                       // 退開走完，進入慢速接近
    TEST_ASSERT_EQUAL_UINT32(60, r.motor.currentHalfPeriodUs());   // 慢速
}

int main(int, char**) {
    UNITY_BEGIN();
    RUN_TEST(test_homing_starts_busy);
    RUN_TEST(test_homing_seeks_then_backs_off_then_reseeks);
    RUN_TEST(test_homing_backs_off_first_if_already_on_switch);
    RUN_TEST(test_homing_fails_when_switch_never_triggers);
    RUN_TEST(test_homing_fails_if_slow_seek_misses);
    RUN_TEST(test_homing_uses_slow_speed_for_second_approach);
    return UNITY_END();
}
