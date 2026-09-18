/*
 * 樣本推送台 + 相機拍照 整合控制（Arduino UNO + CNC Shield）
 *
 * 架構：Arduino 負責所有運動與安全邏輯；PC 端 Python 只負責「收到 CAPTURE
 * 就拍一張、回 OK」。就算 PC 當掉，Arduino 該停的還是會停。
 *
 * 與原始 system.ino 的差別：全部改成非阻塞。原版用 for 迴圈把幾萬步跑完才
 * 返回，期間 loop() 完全停住 —— 自鎖開關讀不到、限位反應不了、Serial 收不到。
 * 現在每輪 loop 只推進一小步，所以停止鈕隨時有反應，而且運動邏輯全部可以在
 * PC 上跑單元測試 (pio test -e native)。
 */
#include <Arduino.h>
#include "hal/ArduinoGpio.h"
#include "hal/ArduinoInputPin.h"
#include "hal/Dm542Driver.h"
#include "hal/A4988Driver.h"
#include "hal/SerialCapture.h"
#include "hal/FakeCapture.h"     // ★ 模擬用，Python 接上後可移除
#include "Button.h"
#include "Stepper.h"
#include "Homing.h"
#include "Scanner.h"

// ═══════════════════════════════════════════
// 腳位（沿用 system.ino 的 CNC Shield 配置）
// ═══════════════════════════════════════════
static const uint8_t X_STP = 2,  X_DIR = 5;    // 相機 X（A4988）
static const uint8_t Y_STP = 3,  Y_DIR = 6;    // 相機 Y（A4988）
static const uint8_t EN    = 8;                // A4988 致能，LOW = 啟用
static const uint8_t X_LIMIT = 9, Y_LIMIT = 10;          // 原點側（END STOPS）
// 遠端側。D12/D13 在 Shield 上各有兩個等效接點：左下 A.STP/A.DIR，
// 右下 SpnEn/SpnDir —— 銅箔同一條，接哪個都通。A 軸那組跳線帽必須不插。
static const uint8_t X_LIMIT_FAR = 12, Y_LIMIT_FAR = 13;

// 輸送台 → DM542。D4/D7 從 Z.STEP/DIR 引出孔拉線，Z 槽保持空的。
static const uint8_t CONV_PUL = 4, CONV_DIR = 7;

static const uint8_t CONV_BOTTOM    = A0;   // Abort 接點
static const uint8_t CONV_TOP       = 11;   // END STOPS Z+，＝輸送台原點
static const uint8_t SAMPLE_PRESENT = A1;
static const uint8_t IR_SENSOR      = A2;
static const uint8_t START_SW       = A3;

// ═══════════════════════════════════════════
// 訊號極性 —— 一定要照實際硬體核對
// activeLow = true  : 觸發時腳位被拉到 LOW（NO 開關接地、感測器低電位輸出）
// activeLow = false : 觸發時腳位是 HIGH（NC 開關斷開後被 pullup 拉高）
// ═══════════════════════════════════════════
// 六顆限位開關統一 NC 接法（C→GND、NC→訊號腳）：斷線時看起來像被觸發，
// 程式會停機而不是繼續推 —— 失效往安全方向倒。
static const bool X_LIMIT_ACTIVE_LOW        = false;
static const bool Y_LIMIT_ACTIVE_LOW        = false;
static const bool X_LIMIT_FAR_ACTIVE_LOW    = false;
static const bool Y_LIMIT_FAR_ACTIVE_LOW    = false;
static const bool CONV_BOTTOM_ACTIVE_LOW    = false;  // NC 接法（C→GND）
static const bool CONV_TOP_ACTIVE_LOW       = false;  // NC 接法，與底部同型同接法
static const bool SAMPLE_PRESENT_ACTIVE_LOW = true;
static const bool IR_ACTIVE_LOW             = true;
static const bool START_SW_ACTIVE_LOW       = true;

// 馬達方向（實機第一次測試務必手扶著、隨時準備斷電）
static const bool CONV_DOWN = true;    // 輸送台往下（往相機、往底部）
static const bool CONV_UP   = false;

// 相機原點方向：X 原點在 LOW 方向、Y 原點在 HIGH 方向（沿用 system.ino 慣例）
static const bool CAM_X_TOWARD_HOME = false;
static const bool CAM_Y_TOWARD_HOME = true;

// ═══════════════════════════════════════════
// 運動參數
// ═══════════════════════════════════════════
static const uint32_t CONV_PPR = 800;    // DM542 DIP 1/4 微步（2026-09-18 實測確認）
static const uint32_t CAM_PPR  = 400;    // A4988 1/2 微步（MS1 插、MS2 空、MS3 插）
                                         // ★ 動跳線帽就要改這裡，STEPS_PER_COL/ROW 也得重量

// 轉速 = 60e6 / (2 * 半週期us * PPR)。兩軸 PPR 不同，同一個半週期轉速差兩倍，
// 所以歸位速度必須分開設 —— 共用一組會讓其中一軸落進共振帶。
static const uint32_t CONV_HALF_US = 100;  // 輸送台巡航，800 PPR → 375 rpm
static const uint32_t CAM_HALF_US  = 800;  // 相機巡航，400 PPR → 93 rpm

// 歸位：快速接近用來省時間，慢速二次接近決定重現性。
// 相機是皮帶傳動，撞到開關後若還在高速推會跳齒，所以慢速這段要夠慢。
static const uint32_t CONV_HOME_FAST_US = 400;   // 800 PPR → 93 rpm
static const uint32_t CONV_HOME_SLOW_US = 1200;  // 800 PPR → 31 rpm
static const uint32_t CAM_HOME_FAST_US  = 800;   // 400 PPR → 93 rpm
static const uint32_t CAM_HOME_SLOW_US  = 2500;  // 400 PPR → 30 rpm
static const uint32_t HOME_BACKOFF = 200;

// 加速斜坡。輸送台帶負載、慣量大，斜坡放長一點。
static const uint32_t CONV_RAMP_START_US = 2000;
static const uint32_t CONV_RAMP_STEPS    = 400;
static const uint32_t CAM_RAMP_START_US  = 2000;
static const uint32_t CAM_RAMP_STEPS     = 200;

// 拍照起始位置（相對於 Homing 原點的步數，方向與 CAM_X/Y_TOWARD_HOME 相反側）。
// 2026-09-18 實測全行程：X = 16515 步、Y = 12108 步。
// 掃描長方形 X 需 (COLS-1)*STEPS_PER_COL = 12240、Y 需 (ROWS-1)*STEPS_PER_ROW = 6240，
// 置中後兩端各留 (全行程 - 掃描需要) / 2：
static const uint32_t PHOTO_X_STEPS = 2137;   // X 兩端餘裕僅 2137 步，累積誤差要靠定期歸位清掉
static const uint32_t PHOTO_Y_STEPS = 2934;

// ═══════════════════════════════════════════
// ★ 暫行模擬設定 —— A1／A2 感測器還沒到貨
// ═══════════════════════════════════════════
// 兩顆感測器到貨接好後，把這兩個改回 false，其餘程式碼不用動。
static const bool SIMULATE_SAMPLE_ALWAYS = true;   // 當作頂部永遠有樣本
static const bool SIMULATE_FEED_BY_STEPS = true;   // 不等 A2，改走固定步數

// 實測輸送台全行程 = 64000 步（2026-09-17），取中點當暫定拍照位置。
// 樣本實際停的位置若偏了，改這個數字即可。
static const uint32_t FEED_STEPS = 32000;

static const uint32_t STEPS_PER_COL = 1360;
static const uint32_t STEPS_PER_ROW = 1040;
static const uint16_t COLS = 10;
static const uint16_t ROWS = 7;            // 樣本盤實際列數

// 逾時保護：超過這個步數還沒等到該等的訊號就判定卡料/故障
// 實測輸送台全行程 = 80 圈 = 64000 步（2026-09-17），這些上限留約 15% 餘裕。
static const uint32_t MAX_STEPS_TO_IR     = 75000UL;
static const uint32_t MAX_STEPS_TO_BOTTOM = 75000UL;
static const uint32_t MAX_STEPS_TO_TOP    = 75000UL;
static const uint32_t MAX_HOMING_STEPS    = 75000UL;

static const long HOMING_EVERY_N_ROUNDS = 2;   // ★ 測試期先設 2，確認誤差累積狀況後再放寬

// ═══════════════════════════════════════════
// 硬體實體
// ═══════════════════════════════════════════
static ArduinoGpio enablePin(EN);

static ArduinoInputPin xLimitPin(X_LIMIT), yLimitPin(Y_LIMIT);
static ArduinoInputPin xLimitFarPin(X_LIMIT_FAR), yLimitFarPin(Y_LIMIT_FAR);
static ArduinoInputPin convBottomPin(CONV_BOTTOM), convTopPin(CONV_TOP);
static ArduinoInputPin samplePin(SAMPLE_PRESENT), irPin(IR_SENSOR), startPin(START_SW);

static Button xLimit(xLimitPin, 5, X_LIMIT_ACTIVE_LOW);
static Button yLimit(yLimitPin, 5, Y_LIMIT_ACTIVE_LOW);
static Button xLimitFar(xLimitFarPin, 5, X_LIMIT_FAR_ACTIVE_LOW);
static Button yLimitFar(yLimitFarPin, 5, Y_LIMIT_FAR_ACTIVE_LOW);
static Button convBottom(convBottomPin, 5, CONV_BOTTOM_ACTIVE_LOW);
static Button convTop(convTopPin, 5, CONV_TOP_ACTIVE_LOW);
static Button samplePresent(samplePin, 20, SAMPLE_PRESENT_ACTIVE_LOW);
static Button irSensor(irPin, 5, IR_ACTIVE_LOW);
static Button startSwitch(startPin, 30, START_SW_ACTIVE_LOW);

static A4988Driver xDrv(X_STP, X_DIR), yDrv(Y_STP, Y_DIR);
static Dm542Driver convDrv(CONV_PUL, CONV_DIR);

static Stepper xMotor(xDrv, CAM_PPR, CAM_HALF_US);
static Stepper yMotor(yDrv, CAM_PPR, CAM_HALF_US);
static Stepper conveyor(convDrv, CONV_PPR, CONV_HALF_US);

// ★ 模擬模式：Python 還沒接上，用 FakeCapture 自己回應，好把整個流程跑完。
//   Python 接好後改回 SerialCapture 那一行即可（Scanner 不用動）。
// static SerialCapture camera(10000);
static FakeCapture camera(300);   // 每格假裝拍 300ms

static Homing xHoming(xMotor, xLimit, CAM_X_TOWARD_HOME,
                      CAM_HOME_FAST_US, CAM_HOME_SLOW_US,
                      HOME_BACKOFF, MAX_HOMING_STEPS);
static Homing yHoming(yMotor, yLimit, CAM_Y_TOWARD_HOME,
                      CAM_HOME_FAST_US, CAM_HOME_SLOW_US,
                      HOME_BACKOFF, MAX_HOMING_STEPS);
static Homing convHoming(conveyor, convTop, CONV_UP,
                         CONV_HOME_FAST_US, CONV_HOME_SLOW_US,
                         HOME_BACKOFF, MAX_HOMING_STEPS);

// 掃描往「遠離 Y 原點」的方向推進 —— Y 原點在 D10 側，所以掃描要往 D13 走。
static Scanner scanner(xMotor, yMotor, camera, COLS, ROWS,
                       STEPS_PER_COL, STEPS_PER_ROW, !CAM_Y_TOWARD_HOME);

// ═══════════════════════════════════════════
// 主狀態機
// ═══════════════════════════════════════════
enum class State : uint8_t {
    Idle,           // 等自鎖開關
    HomeCam, HomeConv,   // HomeCam：X/Y 同時歸位
    ToPhoto,             // 從原點移到拍照起始位置（X/Y 同時）
    CheckSample,    // 頂部還有樣本嗎
    Feed,           // 往下推，等紅外線
    Image,          // 相機掃描
    BackToPhoto,    // 掃完回拍照位置 standby
    ReHome,              // 滿 N 個樣本了，重新歸位（X/Y 同時）
    ToBottom,       // 推到底部
    ToTop,          // 回頂部
    NoSample,       // 沒樣本了，待命
    Fault           // 故障停機
};

static State state = State::Idle;
// 相機回程時輸送台是否已先行下推 —— 避免每輪 loop 重複排入移動。
static bool  convDescending = false;
static long  roundCount = 0;

static void fail(const __FlashStringHelper* reason) {
    xMotor.stop(); yMotor.stop(); conveyor.stop();
    enablePin.write(true);            // EN = HIGH，關掉 A4988 輸出
    state = State::Fault;
    Serial.print(F("FAULT:"));
    Serial.println(reason);
}

// 掃完一輪後往下推。ReHome 完成和「不需要 ReHome」兩條路都走這裡。
static void startDescentToBottom() {
    Serial.println(F("TO_BOTTOM"));
    // Homing 結束會把速度留在 CONV_HOME_FAST_US，所以這裡明確設回巡航值。
    conveyor.setHalfPeriodUs(CONV_HALF_US);
    conveyor.moveUntilSignal(MAX_STEPS_TO_BOTTOM, CONV_DOWN);
    state = State::ToBottom;
}

// 從原點移到拍照起始位置。開機 Homing 完和每次 ReHoming 完都走這裡。
// 方向與「朝原點」相反 —— 原點在哪一側由 CAM_X/Y_TOWARD_HOME 決定。
static void startMoveToPhoto() {
    Serial.println(F("TO_PHOTO_POS"));
    // X/Y 互相獨立，一起排入 —— 兩軸同時走，不用排隊。
    if (PHOTO_X_STEPS > 0) xMotor.moveSteps(PHOTO_X_STEPS, !CAM_X_TOWARD_HOME);
    if (PHOTO_Y_STEPS > 0) yMotor.moveSteps(PHOTO_Y_STEPS, !CAM_Y_TOWARD_HOME);
    state = State::ToPhoto;
}

void setup() {
    Serial.begin(115200);

    enablePin.init();
    xLimitPin.init(); yLimitPin.init();
    xLimitFarPin.init(); yLimitFarPin.init();
    convBottomPin.init(); convTopPin.init();
    samplePin.init(); irPin.init(); startPin.init();

    xDrv.init(); yDrv.init(); convDrv.init();

    xMotor.setRamp(CAM_RAMP_START_US, CAM_RAMP_STEPS);
    yMotor.setRamp(CAM_RAMP_START_US, CAM_RAMP_STEPS);
    conveyor.setRamp(CONV_RAMP_START_US, CONV_RAMP_STEPS);

    enablePin.write(false);           // EN = LOW，啟用 A4988
    Serial.println(F("READY"));
}

void loop() {
    const uint32_t nowMs = millis();
    const uint32_t nowUs = micros();

    // --- 所有輸入與馬達每輪都要 tick ---------------------------------
    startSwitch.tick(nowMs);
    xLimit.tick(nowMs);        yLimit.tick(nowMs);
    xLimitFar.tick(nowMs);     yLimitFar.tick(nowMs);
    convBottom.tick(nowMs);    convTop.tick(nowMs);
    samplePresent.tick(nowMs); irSensor.tick(nowMs);
    camera.poll(nowMs);

    xMotor.tick(nowUs);
    yMotor.tick(nowUs);
    conveyor.tick(nowUs);

    // --- 輸送台端點硬性保護 ------------------------------------------
    // 不論處在哪個狀態，只要壓到端點限位就立刻停 —— 這是全域的最後一道防線。
    // 之前只在特定狀態裡檢查，結果輸送台在 Image 狀態下與相機回程並行下推時
    // 沒人看底部開關，一路把 A0 撞掉。方向判斷用 direction()：只有正在
    // 往那一端走才停，否則剛離開端點時會被自己的限位卡住動不了。
    if (conveyor.isMoving()) {
        const bool goingDown = (conveyor.direction() == CONV_DOWN);
        if (goingDown ? convBottom.isTriggered() : convTop.isTriggered()) {
            conveyor.stop();
        }
    }

    // --- 自鎖開關關掉 -> 立刻停止一切 --------------------------------
    // 原版要等當前動作跑完才會發現開關關了；現在是即時的。
    if (!startSwitch.isDown()) {
        if (state != State::Idle) {
            xMotor.stop(); yMotor.stop(); conveyor.stop();
            enablePin.write(false);
            Serial.println(F("STOPPED"));
            state = State::Idle;
            roundCount = 0;
        }
        return;
    }

    switch (state) {

    case State::Idle:
        // 壓在遠端開關上就不能歸位 —— Homing 的第一步會誤判成壓在原點側，
        // 往反方向退開，也就是朝遠端再推一段。先要求人工推離。
        if (xLimitFar.isTriggered() || yLimitFar.isTriggered()) {
            fail(F("CAM_ON_FAR_LIMIT"));
            break;
        }
        Serial.println(F("HOMING:CAM"));
        // X/Y 兩軸各有自己的馬達與限位，歸位可以同時進行。
        xHoming.start();
        yHoming.start();
        state = State::HomeCam;
        break;

    case State::HomeCam:          // X/Y 同時歸位，兩個都完成才往下走
        xHoming.tick();
        yHoming.tick();
        if (xHoming.result() == Homing::Result::Failed) { fail(F("CAM_HOME_X")); break; }
        if (yHoming.result() == Homing::Result::Failed) { fail(F("CAM_HOME_Y")); break; }
        if (xHoming.result() == Homing::Result::Done &&
            yHoming.result() == Homing::Result::Done) {
            Serial.println(F("HOMED:CAM"));
            Serial.println(F("HOMING:CONV"));
            convHoming.start();
            state = State::HomeConv;
        }
        break;

    case State::HomeConv:
        convHoming.tick();
        if (convHoming.result() == Homing::Result::Failed) { fail(F("CONV_HOME_TIMEOUT")); break; }
        if (convHoming.result() == Homing::Result::Done) {
            roundCount = 0;
            startMoveToPhoto();
        }
        break;

    case State::ToPhoto:
        // X/Y 在 startMoveToPhoto() 已一起排入，兩個都停了才算到位。
        if (xMotor.isMoving() || yMotor.isMoving()) break;
        Serial.println(F("AT_PHOTO_POS"));
        state = State::CheckSample;
        break;

    case State::CheckSample:
        if (!SIMULATE_SAMPLE_ALWAYS && !samplePresent.isTriggered()) {
            Serial.println(F("NO_SAMPLE"));
            state = State::NoSample;
            break;
        }
        Serial.println(F("FEEDING"));
        if (SIMULATE_FEED_BY_STEPS) {
            // 沒有 A2，改走固定步數。底部限位仍會在 Feed 裡擋住，不會撞到底。
            conveyor.setHalfPeriodUs(CONV_HALF_US);
            conveyor.moveSteps(FEED_STEPS, CONV_DOWN);
        } else {
            conveyor.setHalfPeriodUs(CONV_HALF_US);
            conveyor.moveUntilSignal(MAX_STEPS_TO_IR, CONV_DOWN);
        }
        state = State::Feed;
        break;

    case State::Feed:
        // 走固定步數的模擬模式：步數走完就當作到位。
        // 底部限位仍然檢查 —— 步數設太大時才不會一路撞到底。
        if (SIMULATE_FEED_BY_STEPS) {
            if (convBottom.isTriggered()) {
                conveyor.stop();
                fail(F("FEED_HIT_BOTTOM"));   // FEED_STEPS 設太大了
                break;
            }
            if (!conveyor.isMoving()) {
                Serial.print(F("FED_STEPS:"));
                Serial.println(FEED_STEPS);
                Serial.println(F("IMAGING:START"));
                convDescending = false;
                scanner.start();
                state = State::Image;
            }
            break;
        }
        // 正常模式：只看紅外線，不看底部開關
        if (irSensor.isTriggered()) {
            conveyor.stop();
            Serial.println(F("IR_DETECTED"));
            Serial.println(F("IMAGING:START"));
            convDescending = false;
            scanner.start();
            state = State::Image;
            break;
        }
        if (!conveyor.isMoving()) { fail(F("NO_IR_JAM")); }
        break;

    case State::Image:
        // 掃描過程中不該碰到任何限位 —— 原點側和遠端側都算
        if (xLimit.isTriggered()    || yLimit.isTriggered() ||
            xLimitFar.isTriggered() || yLimitFar.isTriggered()) scanner.abortOnLimit();
        scanner.tick();
        if (scanner.result() == Scanner::Result::CaptureFailed) { fail(F("CAPTURE_TIMEOUT")); break; }
        if (scanner.result() == Scanner::Result::LimitHit)      { fail(F("CAM_LIMIT_HIT"));   break; }
        // 最後一格拍完、相機一進入回程就讓輸送台先動 —— 三顆馬達各自獨立，
        // loop() 每輪都會推進，兩段動作重疊可省下整個回程的時間。
        if (scanner.isReturning() && !convDescending) {
            convDescending = true;
            Serial.println(F("TO_BOTTOM (與相機回程同時進行)"));
            conveyor.setHalfPeriodUs(CONV_HALF_US);
            conveyor.moveUntilSignal(MAX_STEPS_TO_BOTTOM, CONV_DOWN);
        }
        if (scanner.result() == Scanner::Result::Done) {
            Serial.println(F("IMAGING:DONE"));
            state = State::BackToPhoto;
        }
        break;

    case State::BackToPhoto:
        // 等相機真的回到拍照起點才進 ToBottom —— 輸送台此時已經在往下走了。
        // 兩者都還沒完成時就停在這個狀態，但馬達照樣在動。
        // 輸送台若先到底，loop 開頭的全域保護已經把它停住了；
        // ToBottom 會看 convBottom 仍被壓著而正常計數，不會漏掉。
        if (xMotor.isMoving() || yMotor.isMoving()) break;
        Serial.println(F("STANDBY_AT_PHOTO"));
        state = State::ToBottom;
        break;

    case State::ReHome:           // X/Y 同時歸位
        xHoming.tick();
        yHoming.tick();
        if (xHoming.result() == Homing::Result::Failed) { fail(F("CAM_HOME_X")); break; }
        if (yHoming.result() == Homing::Result::Failed) { fail(F("CAM_HOME_Y")); break; }
        if (xHoming.result() == Homing::Result::Done &&
            yHoming.result() == Homing::Result::Done) {
            Serial.println(F("HOMED:CAM"));
            startMoveToPhoto();   // 歸位完回拍照位置，再接著下一個樣本
        }
        break;

    case State::ToBottom:
        // 這階段只看底部開關，完全不看紅外線 -> 樣本還壓在感測器上也不會卡死
        if (convBottom.isTriggered()) {
            conveyor.stop();
            // 推到底代表這個樣本完成了 —— 計數在這裡，不在掃描完成時。
            ++roundCount;
            Serial.print(F("AT_BOTTOM · SAMPLE_DONE:"));
            Serial.println(roundCount);
            Serial.println(F("RETURN_TOP"));
            conveyor.setHalfPeriodUs(CONV_HALF_US);
            conveyor.moveUntilSignal(MAX_STEPS_TO_TOP, CONV_UP);
            state = State::ToTop;
            break;
        }
        if (!conveyor.isMoving()) { fail(F("NO_BOTTOM_LIMIT")); }
        break;

    case State::ToTop:
        if (convTop.isTriggered()) {
            conveyor.stop();
            Serial.print(F("ROUND_DONE:"));
            Serial.println(roundCount);

            // 滿 N 個樣本就重新歸位一次，把累積的誤差清掉。
            // 另外檢查丟步：相機此刻應該停在拍照位置，不該壓著原點開關。
            bool scheduled = (roundCount % HOMING_EVERY_N_ROUNDS == 0);
            bool lostSteps = (PHOTO_X_STEPS > 0 && xLimit.isTriggered()) ||
                             (PHOTO_Y_STEPS > 0 && yLimit.isTriggered());

            if (scheduled || lostSteps) {
                Serial.println(scheduled ? F("SCHEDULED_HOMING") : F("WARN:LOST_STEPS"));
                xHoming.start();
                state = State::ReHome;
            } else {
                state = State::CheckSample;
            }
            break;
        }
        if (!conveyor.isMoving()) { fail(F("NO_TOP_LIMIT")); }
        break;

    case State::NoSample:
    case State::Fault:
        // 停在這裡，把自鎖開關關掉再開才會重來
        break;
    }
}
