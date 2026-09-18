#include <unity.h>
#include "IGpio.h"
#include "Button.h"

// 假的腳位：可以任意擺弄電氣準位，模擬彈跳。
class FakePin : public IGpio {
public:
    void init() override           { initCalled = true; }
    void write(bool) override      {}
    bool read() const override      { return level; }

    bool initCalled = false;
    bool level      = true;   // INPUT_PULLUP 沒按時是 HIGH
};

// 模擬「忘記呼叫 init()」的腳位：沒有啟用內部上拉，準位是浮接的。
// 真實硬體上這會讀到隨機值 —— 這裡固定回 false (LOW)，模擬最常見的結果：
// NC 極限開關會被永遠讀成「正常」，按下去程式完全沒反應。
class FloatingPin : public IGpio {
public:
    void init() override      { initialised = true; }
    void write(bool) override {}
    bool read() const override { return initialised ? level : false; }

    bool initialised = false;
    bool level       = true;
};

// 按下 = 拉到 LOW
static void press(FakePin& p)   { p.level = false; }
static void release(FakePin& p) { p.level = true; }

void test_starts_up_not_pressed(void) {
    FakePin p; Button b(p, 30);
    b.tick(0);
    TEST_ASSERT_FALSE(b.isDown());
    TEST_ASSERT_FALSE(b.wasPressed());
}

void test_ignores_press_shorter_than_debounce(void) {
    FakePin p; Button b(p, 30);
    b.tick(0);
    press(p);
    b.tick(10);           // 準位變了，開始計時
    release(p);
    b.tick(20);           // 還沒滿 30ms 就彈回去 —— 雜訊，不採信
    b.tick(100);
    TEST_ASSERT_FALSE(b.isDown());
    TEST_ASSERT_FALSE(b.wasPressed());
}

void test_accepts_press_after_debounce(void) {
    FakePin p; Button b(p, 30);
    b.tick(0);
    press(p);
    b.tick(10);           // 偵測到變化
    b.tick(39);           // 還沒滿 30ms
    TEST_ASSERT_FALSE(b.isDown());
    b.tick(40);           // 穩定超過 30ms，採信
    TEST_ASSERT_TRUE(b.isDown());
    TEST_ASSERT_TRUE(b.wasPressed());
}

// 真實彈跳：接觸瞬間快速通斷數次，最後穩定在按下。
void test_survives_contact_bounce(void) {
    FakePin p; Button b(p, 30);
    b.tick(0);
    press(p);   b.tick(1);
    release(p); b.tick(2);
    press(p);   b.tick(3);
    release(p); b.tick(5);
    press(p);   b.tick(6);    // 最後穩定在按下
    b.tick(35);               // 距離最後一次變化 (t=6) 還不到 30ms
    TEST_ASSERT_FALSE(b.wasPressed());
    b.tick(36);               // 6 + 30 = 36，到了
    TEST_ASSERT_TRUE(b.isDown());
    TEST_ASSERT_TRUE(b.wasPressed());
}

// wasPressed 是「邊緣」，只在按下的那一次 tick 為 true。
void test_pressed_edge_fires_once_only(void) {
    FakePin p; Button b(p, 30);
    b.tick(0);
    press(p); b.tick(10); b.tick(50);
    TEST_ASSERT_TRUE(b.wasPressed());
    b.tick(60);               // 還按著，但不該再觸發一次
    TEST_ASSERT_FALSE(b.wasPressed());
    TEST_ASSERT_TRUE(b.isDown());
}

void test_release_does_not_fire_pressed_edge(void) {
    FakePin p; Button b(p, 30);
    b.tick(0);
    press(p); b.tick(10); b.tick(50);
    release(p);
    b.tick(60); b.tick(100);
    TEST_ASSERT_FALSE(b.isDown());
    TEST_ASSERT_FALSE(b.wasPressed());   // 放開不算按下
}

void test_two_separate_presses_fire_twice(void) {
    FakePin p; Button b(p, 30);
    b.tick(0);
    press(p);   b.tick(10);  b.tick(50);
    TEST_ASSERT_TRUE(b.wasPressed());
    release(p); b.tick(60);  b.tick(100);
    press(p);   b.tick(110); b.tick(150);
    TEST_ASSERT_TRUE(b.wasPressed());
}

// millis() 溢位，去彈跳的計時仍要正確。
void test_survives_millis_overflow(void) {
    FakePin p; Button b(p, 30);
    uint32_t nearMax = 0xFFFFFFF0;
    b.tick(nearMax);
    press(p);
    b.tick(nearMax + 5);      // 偵測到變化
    b.tick(nearMax + 20);     // 還沒滿 30ms
    TEST_ASSERT_FALSE(b.isDown());
    b.tick(nearMax + 35);     // 跨過溢位點，仍應正確採信
    TEST_ASSERT_TRUE(b.isDown());
}


// --- NC 極限開關 (activeLow = false) ----------------------------------
// 接點平時閉合接到 GND (LOW = 正常)，壓下時斷開 (HIGH = 觸發)。

void test_nc_switch_idle_is_not_triggered(void) {
    FakePin p; Button sw(p, 30, false);
    p.level = false;              // C-NC 導通接地 = 正常運行
    sw.tick(0); sw.tick(100);
    TEST_ASSERT_FALSE(sw.isTriggered());
}

void test_nc_switch_triggers_when_contact_opens(void) {
    FakePin p; Button sw(p, 30, false);
    p.level = false;
    sw.tick(0); sw.tick(100);
    TEST_ASSERT_FALSE(sw.isTriggered());

    p.level = true;               // 滑塊壓到開關，接點斷開
    sw.tick(110);
    sw.tick(150);
    TEST_ASSERT_TRUE(sw.isTriggered());
    TEST_ASSERT_TRUE(sw.justTriggered());
}

// 失效安全：線鬆脫時腳位被內部上拉到 HIGH，效果等同「觸發」。
void test_nc_switch_treats_broken_wire_as_triggered(void) {
    FakePin p; Button sw(p, 30, false);
    p.level = false;
    sw.tick(0); sw.tick(100);
    p.level = true;               // 斷線 —— 電氣上跟壓下開關無法區分
    sw.tick(110); sw.tick(150);
    TEST_ASSERT_TRUE(sw.isTriggered());
}

// 對照組：同樣的斷線情境，NO 開關會被讀成「正常」—— 這就是不該用 NO 的原因。
void test_no_switch_would_miss_broken_wire(void) {
    FakePin p; Button sw(p, 30, true);   // activeLow = NO 接法
    p.level = true;                      // 斷線，上拉到 HIGH
    sw.tick(0); sw.tick(100);
    TEST_ASSERT_FALSE(sw.isTriggered()); // 被當成一切正常
}

void test_nc_switch_debounces(void) {
    FakePin p; Button sw(p, 30, false);
    p.level = false;
    sw.tick(0); sw.tick(100);
    p.level = true;
    sw.tick(110);
    sw.tick(135);                 // 還沒滿 30ms
    TEST_ASSERT_FALSE(sw.isTriggered());
    sw.tick(140);
    TEST_ASSERT_TRUE(sw.isTriggered());
}


// 回歸測試：漏掉 init() 曾讓極限開關完全失效 —— D5 停在預設的 INPUT 浮接
// 狀態，沒有上拉電阻，NC 開關按下去讀不到變化，馬達不會停。
void test_uninitialised_pin_never_triggers_nc_switch(void) {
    FloatingPin p; Button sw(p, 20, false);
    p.level = true;               // 開關壓下，接點斷開
    sw.tick(0); sw.tick(100);
    TEST_ASSERT_FALSE(sw.isTriggered());   // 沒 init 就偵測不到 —— bug 的樣子

    p.init();                     // 正確初始化之後
    sw.tick(200); sw.tick(300);
    TEST_ASSERT_TRUE(sw.isTriggered());    // 才讀得到
}

int main(int, char**) {
    UNITY_BEGIN();
    RUN_TEST(test_starts_up_not_pressed);
    RUN_TEST(test_ignores_press_shorter_than_debounce);
    RUN_TEST(test_accepts_press_after_debounce);
    RUN_TEST(test_survives_contact_bounce);
    RUN_TEST(test_pressed_edge_fires_once_only);
    RUN_TEST(test_release_does_not_fire_pressed_edge);
    RUN_TEST(test_two_separate_presses_fire_twice);
    RUN_TEST(test_survives_millis_overflow);
    RUN_TEST(test_nc_switch_idle_is_not_triggered);
    RUN_TEST(test_nc_switch_triggers_when_contact_opens);
    RUN_TEST(test_nc_switch_treats_broken_wire_as_triggered);
    RUN_TEST(test_no_switch_would_miss_broken_wire);
    RUN_TEST(test_nc_switch_debounces);
    RUN_TEST(test_uninitialised_pin_never_triggers_nc_switch);
    return UNITY_END();
}
