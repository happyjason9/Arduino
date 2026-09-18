#include <unity.h>
#include "IGpio.h"
#include "Blinker.h"

// 假的 GPIO：不碰硬體，只記錄被呼叫的狀況。
class FakeGpio : public IGpio {
public:
    void init() override           { initCalled = true; }
    void write(bool high) override { state = high; ++writeCount; }
    bool read() const override      { return state; }

    bool initCalled = false;
    bool state      = false;
    int  writeCount = 0;
};

void test_starts_off(void) {
    FakeGpio gpio;
    Blinker  b(gpio, 500);
    TEST_ASSERT_FALSE(b.isOn());
}

void test_does_not_toggle_before_period(void) {
    FakeGpio gpio;
    Blinker  b(gpio, 500);
    b.tick(499);
    TEST_ASSERT_FALSE(b.isOn());
    TEST_ASSERT_EQUAL_INT(0, gpio.writeCount);
}

void test_toggles_at_period(void) {
    FakeGpio gpio;
    Blinker  b(gpio, 500);
    b.tick(500);
    TEST_ASSERT_TRUE(b.isOn());
    TEST_ASSERT_TRUE(gpio.state);
}

void test_toggles_back_and_forth(void) {
    FakeGpio gpio;
    Blinker  b(gpio, 500);
    b.tick(500);   TEST_ASSERT_TRUE(b.isOn());
    b.tick(1000);  TEST_ASSERT_FALSE(b.isOn());
    b.tick(1500);  TEST_ASSERT_TRUE(b.isOn());
    TEST_ASSERT_EQUAL_INT(3, gpio.writeCount);
}

// millis() 在約 49.7 天後會溢位歸零 —— 真實世界的 bug 來源。
// 無號數相減讓這個 case 依然正確，這裡把它釘住。
void test_survives_millis_overflow(void) {
    FakeGpio gpio;
    Blinker  b(gpio, 500);
    uint32_t nearMax = 0xFFFFFF00;
    b.tick(nearMax);              // 第一次一定會觸發 (nearMax - 0 >= 500)
    int before = gpio.writeCount;
    b.tick(nearMax + 499);        // 還沒到
    TEST_ASSERT_EQUAL_INT(before, gpio.writeCount);
    b.tick(nearMax + 500);        // 溢位後仍應正確觸發
    TEST_ASSERT_EQUAL_INT(before + 1, gpio.writeCount);
}


void test_set_period_changes_blink_rate(void) {
    FakeGpio gpio;
    Blinker  b(gpio, 500);
    b.tick(500);                  // 第一次切換
    TEST_ASSERT_TRUE(b.isOn());
    b.setPeriod(100);             // 改成快閃
    TEST_ASSERT_EQUAL_UINT32(100, b.period());
    b.tick(599);                  // 距上次切換還不到 100ms
    TEST_ASSERT_TRUE(b.isOn());
    b.tick(600);                  // 到了
    TEST_ASSERT_FALSE(b.isOn());
}

int main(int, char**) {
    UNITY_BEGIN();
    RUN_TEST(test_starts_off);
    RUN_TEST(test_does_not_toggle_before_period);
    RUN_TEST(test_toggles_at_period);
    RUN_TEST(test_toggles_back_and_forth);
    RUN_TEST(test_survives_millis_overflow);
    RUN_TEST(test_set_period_changes_blink_rate);
    return UNITY_END();
}
