/*
 * 相機 X/Y 行程量測 + 手動點動
 *
 * 用法：複製成 src/main.cpp 再燒錄。正式韌體備份在 main_reg2.cpp。
 *
 * 目的：量出 X/Y 兩端限位之間的總行程，用來算拍照起始位置：
 *
 *     PHOTO_X_STEPS = (X 總行程 - (COLS-1) * STEPS_PER_COL) / 2
 *     PHOTO_Y_STEPS = (Y 總行程 - (ROWS-1) * STEPS_PER_ROW) / 2
 *
 *   把掃描長方形置中：左右各留一半邊距。程式會直接把算好的值印出來。
 *
 * 全程走歸位慢速（相機 30 rpm）。皮帶傳動撞到限位還在高速推會跳齒，
 * 量出來的步數就不準 —— 這是之前那份測試韌體數字一直變大的原因。
 *
 * 指令（序列埠 115200）：
 *   mx / my   自動量測該軸全行程（先走到原點歸零，再走到遠端）
 *   x 100     X 軸往遠端點動 100 步；x -100 往原點
 *   y 50      Y 軸點動，同上
 *   p         印出目前座標（以原點為 0）
 *   z         把目前位置當成 0（手動對準後歸零用）
 *   ?         狀態與說明
 *
 * 安全：任何移動前都會檢查該方向的限位是否已被壓住，壓住就拒絕移動。
 */
#include <Arduino.h>
#include "hal/ArduinoGpio.h"
#include "hal/ArduinoInputPin.h"
#include "hal/A4988Driver.h"
#include "Button.h"
#include "Stepper.h"

static const uint8_t EN    = 8;
static const uint8_t X_STP = 2,  X_DIR = 5;
static const uint8_t Y_STP = 3,  Y_DIR = 6;
static const uint8_t X_LIMIT     = 9,  Y_LIMIT     = 10;
static const uint8_t X_LIMIT_FAR = 12, Y_LIMIT_FAR = 13;

static const bool LIMIT_ACTIVE_LOW = false;   // NC 接法

static const bool X_TOWARD_HOME = false;
static const bool Y_TOWARD_HOME = true;

static const uint32_t CAM_PPR = 400;

// 量測全程用慢速，跟正式韌體的 CAM_HOME_SLOW_US 一致（400 PPR → 30 rpm）。
static const uint32_t MEASURE_HALF_US = 2500;
static const uint32_t JOG_HALF_US     = 1200;   // 點動可以快一點
static const uint32_t MAX_TRAVEL      = 200000UL;

// 掃描路徑尺寸 —— 與正式韌體的常數保持一致
static const uint32_t STEPS_PER_COL = 1360;
static const uint32_t STEPS_PER_ROW = 1040;
static const uint16_t COLS = 10;
static const uint16_t ROWS = 7;

static ArduinoGpio     enablePin(EN);
static ArduinoInputPin xLimitPin(X_LIMIT),   yLimitPin(Y_LIMIT);
static ArduinoInputPin xFarPin(X_LIMIT_FAR), yFarPin(Y_LIMIT_FAR);

static Button xLimit(xLimitPin, 5, LIMIT_ACTIVE_LOW);
static Button yLimit(yLimitPin, 5, LIMIT_ACTIVE_LOW);
static Button xFar(xFarPin, 5, LIMIT_ACTIVE_LOW);
static Button yFar(yFarPin, 5, LIMIT_ACTIVE_LOW);

static A4988Driver xDrv(X_STP, X_DIR);
static A4988Driver yDrv(Y_STP, Y_DIR);
static Stepper     xMotor(xDrv, CAM_PPR, JOG_HALF_US);
static Stepper     yMotor(yDrv, CAM_PPR, JOG_HALF_US);

// 目前座標：以原點側限位為 0，往遠端為正。
static long xPos = 0, yPos = 0;

// 量測流程
enum class Phase : uint8_t { None, SeekHome, SeekFar };
static Phase   phase   = Phase::None;
static bool    measX   = true;
static uint32_t stepsAtPhase = 0;

// 點動時記住方向，結束才更新座標
static bool  jogging = false;
static bool  jogX = true;
static long  jogSigned = 0;
static uint32_t jogStepsAtStart = 0;

static Stepper& mot(bool isX)      { return isX ? xMotor : yMotor; }
static Button&  homeSw(bool isX)   { return isX ? xLimit : yLimit; }
static Button&  farSw(bool isX)    { return isX ? xFar   : yFar;   }
static bool     homeDir(bool isX)  { return isX ? X_TOWARD_HOME : Y_TOWARD_HOME; }

static void printPos() {
    Serial.print(F("   X = ")); Serial.print(xPos);
    Serial.print(F("    Y = ")); Serial.println(yPos);
}

static void printLimits() {
    Serial.print(F("   D9 X原點 "));
    Serial.print(xLimit.isTriggered() ? F("[壓]") : F("[放]"));
    Serial.print(F("   D12 X遠端 "));
    Serial.print(xFar.isTriggered() ? F("[壓]") : F("[放]"));
    Serial.print(F("   D10 Y原點 "));
    Serial.print(yLimit.isTriggered() ? F("[壓]") : F("[放]"));
    Serial.print(F("   D13 Y遠端 "));
    Serial.println(yFar.isTriggered() ? F("[壓]") : F("[放]"));
}

// 量到全行程後，算出置中的拍照起始位置。
static void reportTravel(bool isX, long travel) {
    const uint32_t span = isX ? (uint32_t)(COLS - 1) * STEPS_PER_COL
                              : (uint32_t)(ROWS - 1) * STEPS_PER_ROW;
    Serial.println();
    Serial.print(F("=== "));
    Serial.print(isX ? F("X") : F("Y"));
    Serial.println(F(" 軸量測完成 ==="));
    Serial.print(F("  全行程       = ")); Serial.print(travel); Serial.println(F(" 步"));
    Serial.print(F("  掃描需要     = ")); Serial.print(span);   Serial.println(F(" 步"));

    if ((uint32_t)travel <= span) {
        Serial.println(F("  !! 行程不夠掃描用 —— 檢查 COLS/ROWS 或每格步數是否正確"));
    } else {
        const long photo = ((long)travel - (long)span) / 2;
        Serial.print(F("  → PHOTO_"));
        Serial.print(isX ? F("X") : F("Y"));
        Serial.print(F("_STEPS = "));
        Serial.println(photo);
        Serial.println(F("    （掃描長方形置中，左右各留一半邊距）"));
    }
    Serial.println();
}

static bool startMove(bool isX, bool toHome, uint32_t maxSteps, uint32_t halfUs) {
    if (toHome ? homeSw(isX).isTriggered() : farSw(isX).isTriggered()) {
        Serial.print(F("!! "));
        Serial.print(toHome ? F("原點") : F("遠端"));
        Serial.println(F("側限位已壓住，不能往該方向移動"));
        return false;
    }
    mot(isX).setHalfPeriodUs(halfUs);
    mot(isX).moveUntilSignal(maxSteps, toHome ? homeDir(isX) : !homeDir(isX));
    return true;
}

void setup() {
    Serial.begin(115200);
    while (!Serial) { ; }
    delay(200);

    enablePin.init();
    xLimitPin.init(); yLimitPin.init();
    xFarPin.init();   yFarPin.init();
    xDrv.init(); yDrv.init();
    enablePin.write(false);          // LOW = 啟用 A4988

    Serial.println();
    Serial.println(F("=== X/Y 行程量測 + 點動 ==="));
    Serial.println();
    Serial.println(F("  mx / my   自動量測該軸全行程（會先回原點）"));
    Serial.println(F("  x 100     X 往遠端 100 步   x -100 往原點"));
    Serial.println(F("  y 50      Y 軸點動"));
    Serial.println(F("  p         印出座標    z 把目前位置歸零"));
    Serial.println(F("  ?         狀態"));
    Serial.println();
    Serial.println(F("建議：先 mx 再 my，把印出的 PHOTO_*_STEPS 抄下來。"));
    Serial.println();
    printLimits();
}

void loop() {
    const uint32_t nowMs = millis();
    const uint32_t nowUs = micros();

    xLimit.tick(nowMs); yLimit.tick(nowMs);
    xFar.tick(nowMs);   yFar.tick(nowMs);
    xMotor.tick(nowUs); yMotor.tick(nowUs);

    // ── 量測流程 ────────────────────────────────────
    if (phase == Phase::SeekHome) {
        if (homeSw(measX).isTriggered()) {
            mot(measX).stop();
            if (measX) xPos = 0; else yPos = 0;
            Serial.println(F("   到原點，歸零。開始往遠端量測..."));
            stepsAtPhase = mot(measX).stepsTaken();
            if (startMove(measX, false, MAX_TRAVEL, MEASURE_HALF_US)) {
                phase = Phase::SeekFar;
            } else {
                phase = Phase::None;
            }
        } else if (!mot(measX).isMoving()) {
            Serial.println(F("!! 找不到原點限位 —— 檢查接線或方向"));
            phase = Phase::None;
        }
        return;
    }

    if (phase == Phase::SeekFar) {
        if (farSw(measX).isTriggered()) {
            mot(measX).stop();
            const long travel = (long)(mot(measX).stepsTaken() - stepsAtPhase);
            if (measX) xPos = travel; else yPos = travel;
            reportTravel(measX, travel);
            phase = Phase::None;
        } else if (!mot(measX).isMoving()) {
            Serial.println(F("!! 走完上限還沒碰到遠端限位 —— 檢查接線"));
            phase = Phase::None;
        }
        return;
    }

    // ── 點動結束，更新座標 ──────────────────────────
    if (jogging && !mot(jogX).isMoving()) {
        jogging = false;
        const long moved = (long)(mot(jogX).stepsTaken() - jogStepsAtStart);
        const long delta = (jogSigned >= 0) ? moved : -moved;
        if (jogX) xPos += delta; else yPos += delta;
        printPos();
    }

    // 點動途中撞到限位就停
    if (jogging) {
        const bool toHome = (jogSigned < 0);
        if (toHome ? homeSw(jogX).isTriggered() : farSw(jogX).isTriggered()) {
            mot(jogX).stop();
            Serial.println(F("   碰到限位，停止"));
        }
    }

    // ── 序列埠指令 ──────────────────────────────────
    if (!Serial.available()) return;
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    if (cmd.length() == 0) return;

    if (cmd == "?") {
        Serial.println(F("--- 狀態 ---"));
        printPos();
        printLimits();
        Serial.print(F("   掃描需要 X "));
        Serial.print((uint32_t)(COLS - 1) * STEPS_PER_COL);
        Serial.print(F(" 步、Y "));
        Serial.print((uint32_t)(ROWS - 1) * STEPS_PER_ROW);
        Serial.println(F(" 步"));
        return;
    }
    if (cmd == "p") { printPos(); return; }
    if (cmd == "z") { xPos = 0; yPos = 0; Serial.println(F(">> 座標歸零")); return; }

    if (cmd == "mx" || cmd == "my") {
        measX = (cmd == "mx");
        Serial.println();
        Serial.print(F(">> 量測 "));
        Serial.print(measX ? F("X") : F("Y"));
        Serial.println(F(" 軸：先回原點..."));
        if (homeSw(measX).isTriggered()) {
            // 已經壓在原點上，直接進入往遠端的階段
            if (measX) xPos = 0; else yPos = 0;
            stepsAtPhase = mot(measX).stepsTaken();
            if (startMove(measX, false, MAX_TRAVEL, MEASURE_HALF_US)) {
                Serial.println(F("   已在原點，直接往遠端量測..."));
                phase = Phase::SeekFar;
            }
        } else if (startMove(measX, true, MAX_TRAVEL, MEASURE_HALF_US)) {
            phase = Phase::SeekHome;
        }
        return;
    }

    // x / y 點動：cmd 形如 "x 100"、"x-100"、"y 50"
    const char axis = cmd.charAt(0);
    if (axis == 'x' || axis == 'X' || axis == 'y' || axis == 'Y') {
        String arg = cmd.substring(1);
        arg.trim();
        const long n = arg.toInt();
        if (n == 0) {
            Serial.println(F(">> 用法：x 100 或 x -100"));
            return;
        }
        jogX      = (axis == 'x' || axis == 'X');
        jogSigned = n;
        const bool toHome = (n < 0);
        const uint32_t steps = (uint32_t)(n < 0 ? -n : n);
        if (toHome ? homeSw(jogX).isTriggered() : farSw(jogX).isTriggered()) {
            Serial.println(F("!! 該方向限位已壓住，不能移動"));
            return;
        }
        jogStepsAtStart = mot(jogX).stepsTaken();
        mot(jogX).setHalfPeriodUs(JOG_HALF_US);
        mot(jogX).moveSteps(steps, toHome ? homeDir(jogX) : !homeDir(jogX));
        jogging = true;
        return;
    }

    Serial.println(F(">> 不認得的指令。可用：mx my x<n> y<n> p z ?"));
}
