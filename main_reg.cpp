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
static const uint8_t X_LIMIT_FAR = 12, Y_LIMIT_FAR = 13;  // 遠端側（A.STP / A.DIR）

static const uint8_t CONV_PUL = 4, CONV_DIR = 7;   // 輸送台 → DM542

static const uint8_t CONV_BOTTOM    = 11;
static const uint8_t CONV_TOP       = A0;
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
static const uint32_t CONV_PPR = 1600;   // DM542 DIP SW5-8 = off/off/on/on
static const uint32_t CAM_PPR  = 400;    // A4988 1/2 微步（MS1 插、MS2 空、MS3 插）
                                         // ★ 動跳線帽就要改這裡，STEPS_PER_COL/ROW 也得重量

static const uint32_t CONV_HALF_US = 800;  // 輸送台巡航半週期
static const uint32_t CAM_HALF_US  = 800;  // 相機巡航半週期
static const uint32_t HOME_FAST_US = 800;
static const uint32_t HOME_SLOW_US = 2500; // 慢速二次接近，提高重現性
static const uint32_t HOME_BACKOFF = 200;

// 加速斜坡。輸送台帶負載、慣量大，斜坡放長一點。
static const uint32_t CONV_RAMP_START_US = 2000;
static const uint32_t CONV_RAMP_STEPS    = 400;
static const uint32_t CAM_RAMP_START_US  = 2000;
static const uint32_t CAM_RAMP_STEPS     = 200;

static const uint32_t STEPS_PER_COL = 1360;
static const uint32_t STEPS_PER_ROW = 1040;
static const uint16_t COLS = 10;
static const uint16_t ROWS = 1;            // ★ 填實際列數

// 逾時保護：超過這個步數還沒等到該等的訊號就判定卡料/故障
static const uint32_t MAX_STEPS_TO_IR     = 100000UL;
static const uint32_t MAX_STEPS_TO_BOTTOM = 100000UL;
static const uint32_t MAX_STEPS_TO_TOP    = 200000UL;
static const uint32_t MAX_HOMING_STEPS    = 200000UL;

static const long HOMING_EVERY_N_ROUNDS = 10;

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

static SerialCapture camera(10000);

static Homing xHoming(xMotor, xLimit, CAM_X_TOWARD_HOME, HOME_FAST_US, HOME_SLOW_US,
                      HOME_BACKOFF, MAX_HOMING_STEPS);
static Homing yHoming(yMotor, yLimit, CAM_Y_TOWARD_HOME, HOME_FAST_US, HOME_SLOW_US,
                      HOME_BACKOFF, MAX_HOMING_STEPS);
static Homing convHoming(conveyor, convTop, CONV_UP, HOME_FAST_US, HOME_SLOW_US,
                         HOME_BACKOFF, MAX_HOMING_STEPS);

static Scanner scanner(xMotor, yMotor, camera, COLS, ROWS,
                       STEPS_PER_COL, STEPS_PER_ROW);

// ═══════════════════════════════════════════
// 主狀態機
// ═══════════════════════════════════════════
enum class State : uint8_t {
    Idle,           // 等自鎖開關
    HomeCamX, HomeCamY, HomeConv,
    CheckSample,    // 頂部還有樣本嗎
    Feed,           // 往下推，等紅外線
    Image,          // 相機掃描
    ReHomeX, ReHomeY,  // 掃完丟步了或排程到了，重新歸位
    ToBottom,       // 推到底部
    ToTop,          // 回頂部
    NoSample,       // 沒樣本了，待命
    Fault           // 故障停機
};

static State state = State::Idle;
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
    conveyor.moveUntilSignal(MAX_STEPS_TO_BOTTOM, CONV_DOWN);
    state = State::ToBottom;
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
        xHoming.start();
        state = State::HomeCamX;
        break;

    case State::HomeCamX:
        xHoming.tick();
        if (xHoming.result() == Homing::Result::Failed) { fail(F("CAM_HOME_X")); break; }
        if (xHoming.result() == Homing::Result::Done) {
            yHoming.start();
            state = State::HomeCamY;
        }
        break;

    case State::HomeCamY:
        yHoming.tick();
        if (yHoming.result() == Homing::Result::Failed) { fail(F("CAM_HOME_Y")); break; }
        if (yHoming.result() == Homing::Result::Done) {
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
            state = State::CheckSample;
        }
        break;

    case State::CheckSample:
        if (!samplePresent.isTriggered()) {
            Serial.println(F("NO_SAMPLE"));
            state = State::NoSample;
            break;
        }
        Serial.println(F("FEEDING"));
        conveyor.moveUntilSignal(MAX_STEPS_TO_IR, CONV_DOWN);
        state = State::Feed;
        break;

    case State::Feed:
        // 這階段只看紅外線，不看底部開關
        if (irSensor.isTriggered()) {
            conveyor.stop();
            Serial.println(F("IR_DETECTED"));
            Serial.println(F("IMAGING:START"));
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
        if (scanner.result() == Scanner::Result::Done) {
            Serial.println(F("IMAGING:DONE"));
            ++roundCount;

            // 免費的丟步檢查：照步數回到原點後，原點開關應該要是觸發狀態。
            // 對不上就代表這輪丟步了，當場修正而不是讓誤差累積下去。
            bool scheduled = (roundCount % HOMING_EVERY_N_ROUNDS == 0);
            bool lostSteps = !xLimit.isTriggered() || !yLimit.isTriggered();

            if (scheduled || lostSteps) {
                Serial.println(scheduled ? F("SCHEDULED_HOMING") : F("WARN:LOST_STEPS"));
                xHoming.start();
                state = State::ReHomeX;
            } else {
                startDescentToBottom();
            }
        }
        break;

    case State::ReHomeX:
        xHoming.tick();
        if (xHoming.result() == Homing::Result::Failed) { fail(F("CAM_HOME_X")); break; }
        if (xHoming.result() == Homing::Result::Done) {
            yHoming.start();
            state = State::ReHomeY;
        }
        break;

    case State::ReHomeY:
        yHoming.tick();
        if (yHoming.result() == Homing::Result::Failed) { fail(F("CAM_HOME_Y")); break; }
        if (yHoming.result() == Homing::Result::Done) {
            Serial.println(F("HOMED:CAM"));
            startDescentToBottom();
        }
        break;

    case State::ToBottom:
        // 這階段只看底部開關，完全不看紅外線 -> 樣本還壓在感測器上也不會卡死
        if (convBottom.isTriggered()) {
            conveyor.stop();
            Serial.println(F("AT_BOTTOM"));
            Serial.println(F("RETURN_TOP"));
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
            state = State::CheckSample;
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
