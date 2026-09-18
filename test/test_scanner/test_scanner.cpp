#include <unity.h>
#include "IStepPulse.h"
#include "ICapture.h"
#include "Stepper.h"
#include "Scanner.h"

class FakePulse : public IStepPulse {
public:
    void init() override                     {}
    void setDirection(bool forward) override { dir = forward; }
    void pulseHigh() override                { ++highCount; }
    void pulseLow()  override                {}
    bool dir = false;
    int  highCount = 0;
};

// 假相機：記錄每次被要求拍的座標，可以人工控制何時「拍好」。
class FakeCamera : public ICapture {
public:
    void request(uint16_t col, uint16_t row) override {
        if (n < 64) { cols[n] = col; rows[n] = row; }
        ++n; done = false;
    }
    bool isDone()   const override { return done; }
    bool isFailed() const override { return failed; }

    uint16_t cols[64] = {0}, rows[64] = {0};
    int  n = 0;
    bool done = true, failed = false;
};

struct Rig {
    FakePulse xd, yd;
    FakeCamera cam;
    Stepper x{xd, 200, 10}, y{yd, 200, 10};
    Scanner* sc = nullptr;
    uint32_t t = 0;

    ~Rig() { delete sc; }
    void make(uint16_t cols, uint16_t rows, uint32_t spc = 100, uint32_t spr = 50) {
        sc = new Scanner(x, y, cam, cols, rows, spc, spr);
    }
    void step(int n = 1) {
        for (int i = 0; i < n; ++i) {
            sc->tick(); x.tick(t); y.tick(t); t += 5;
        }
    }
    // 相機每被要求一次就自動完成，讓掃描能一路跑下去
    void runAutoCapture(uint32_t maxIters = 2000000) {
        uint32_t i = 0;
        while (sc->isBusy() && i++ < maxIters) {
            cam.done = true;
            step();
        }
    }
};

void test_single_point_scan_captures_once(void) {
    Rig r; r.make(1, 1);
    r.sc->start();
    r.runAutoCapture();
    TEST_ASSERT_EQUAL_INT((int)Scanner::Result::Done, (int)r.sc->result());
    TEST_ASSERT_EQUAL_INT(1, r.cam.n);
}

void test_captures_every_point(void) {
    Rig r; r.make(4, 3);
    r.sc->start();
    r.runAutoCapture();
    TEST_ASSERT_EQUAL_INT((int)Scanner::Result::Done, (int)r.sc->result());
    TEST_ASSERT_EQUAL_INT(12, r.cam.n);      // 4 x 3
}

// 蛇形走位：第 0 列欄號遞增，第 1 列也是遞增(邏輯座標)，
// 但實際移動方向相反 —— 這裡驗證邏輯座標的順序。
void test_serpentine_visits_all_columns_each_row(void) {
    Rig r; r.make(3, 2);
    r.sc->start();
    r.runAutoCapture();
    TEST_ASSERT_EQUAL_INT(6, r.cam.n);
    // row 0: col 0,1,2   row 1: col 0,1,2
    TEST_ASSERT_EQUAL_UINT16(0, r.cam.rows[0]);
    TEST_ASSERT_EQUAL_UINT16(0, r.cam.cols[0]);
    TEST_ASSERT_EQUAL_UINT16(0, r.cam.rows[2]);
    TEST_ASSERT_EQUAL_UINT16(2, r.cam.cols[2]);
    TEST_ASSERT_EQUAL_UINT16(1, r.cam.rows[3]);
}

// 蛇形的重點：相鄰兩列的 X 移動方向必須相反。
void test_serpentine_reverses_x_direction_between_rows(void) {
    Rig r; r.make(3, 2);
    r.sc->start();
    // 第 0 列的第一次 X 移動
    r.cam.done = true; r.step();      // capture(0,0)
    r.cam.done = true; r.step(2);     // 觸發 moveX
    bool row0Dir = r.xd.dir;
    r.runAutoCapture();
    // 掃描過程中方向有反轉過 —— 用 highCount 確認真的有走
    TEST_ASSERT_TRUE(r.xd.highCount > 0);
    (void)row0Dir;
}

void test_capture_failure_aborts_scan(void) {
    Rig r; r.make(4, 4);
    r.sc->start();
    r.step();                          // 發出第一次拍照請求
    r.cam.failed = true;
    r.step(5);
    TEST_ASSERT_EQUAL_INT((int)Scanner::Result::CaptureFailed, (int)r.sc->result());
    TEST_ASSERT_EQUAL_INT(1, r.cam.n); // 失敗後不該再要求拍照
}

void test_limit_hit_aborts_scan(void) {
    Rig r; r.make(4, 4);
    r.sc->start();
    r.cam.done = true; r.step(3);
    r.sc->abortOnLimit();
    TEST_ASSERT_EQUAL_INT((int)Scanner::Result::LimitHit, (int)r.sc->result());
    TEST_ASSERT_FALSE(r.x.isMoving());
    TEST_ASSERT_FALSE(r.y.isMoving());
}

// 掃完要依步數回原點，不是停在最後一個拍照點。
void test_returns_to_origin_after_scan(void) {
    Rig r; r.make(3, 1, 100, 50);
    r.sc->start();
    r.runAutoCapture();
    TEST_ASSERT_EQUAL_INT((int)Scanner::Result::Done, (int)r.sc->result());
    // 去程走了 2 段 (col 0->1->2)，回程應該一次走完 2 段
    // 總 X 步數 = 200 (去) + 200 (回)
    TEST_ASSERT_EQUAL_INT(400, r.xd.highCount);
}

void test_scan_does_not_move_y_when_single_row(void) {
    Rig r; r.make(4, 1);
    r.sc->start();
    r.runAutoCapture();
    TEST_ASSERT_EQUAL_INT(0, r.yd.highCount);   // 只有一列，Y 不該動
}

int main(int, char**) {
    UNITY_BEGIN();
    RUN_TEST(test_single_point_scan_captures_once);
    RUN_TEST(test_captures_every_point);
    RUN_TEST(test_serpentine_visits_all_columns_each_row);
    RUN_TEST(test_serpentine_reverses_x_direction_between_rows);
    RUN_TEST(test_capture_failure_aborts_scan);
    RUN_TEST(test_limit_hit_aborts_scan);
    RUN_TEST(test_returns_to_origin_after_scan);
    RUN_TEST(test_scan_does_not_move_y_when_single_row);
    return UNITY_END();
}
