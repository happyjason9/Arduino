/*
 * 相機 X/Y 軸 測試韌體
 *
 * 用法：把這個檔複製成 src/main.cpp 再燒錄。
 *       輸送台的測試版本備份在 test_cpp/test_conveyor.cpp。
 *
 * 跟輸送台測試的差別：X/Y 兩端都有限位開關，所以任何移動前都要先確認
 * 「要去的那一端沒有被壓著」—— 壓著還往那邊推就是撞機。
 *
 * 行為：A3 按下 → 選定的軸在兩端限位之間來回走，每 400 步印一次進度。
 *       碰到任一端自動折返。A3 放開立刻停。
 *
 * 指令（序列埠 115200）：
 *   x / y    切換要測試的軸（預設 x）
 *   數字     改巡航半週期，30-5000us（越小越快）
 *   s        只讀四顆限位的狀態，不動馬達 —— 接線確認用
 *   ?        印出目前設定與狀態
 *
 * 建議順序：先打 s，用手逐顆壓四個開關，確認每顆都有反應且對應正確，
 *           再按 A3 讓馬達動。第一次務必手扶著、隨時準備斷電。
 */
#include <Arduino.h>
#include "hal/ArduinoGpio.h"
#include "hal/ArduinoInputPin.h"
#include "hal/A4988Driver.h"
#include "Button.h"
#include "Stepper.h"

static const uint8_t EN    = 8;                 // A4988 致能，LOW = 啟用
static const uint8_t X_STP = 2,  X_DIR = 5;
static const uint8_t Y_STP = 3,  Y_DIR = 6;

static const uint8_t X_LIMIT     = 9,  Y_LIMIT     = 10;   // 原點側（END STOPS）
static const uint8_t X_LIMIT_FAR = 12, Y_LIMIT_FAR = 13;   // 遠端側（SpnEn/SpnDir）
static const uint8_t START_SW    = A3;

// 六顆限位統一 NC 接法：壓下才斷開 → 讀到 HIGH 算觸發。
static const bool LIMIT_ACTIVE_LOW    = false;
static const bool START_SW_ACTIVE_LOW = true;

// 哪個方向是朝原點側。與 main_reg2.cpp 一致。
static const bool X_TOWARD_HOME = false;
static const bool Y_TOWARD_HOME = true;

static const uint32_t CAM_PPR = 400;    // A4988 1/2 微步

// 400 PPR 下：800us = 93 rpm、400us = 187 rpm、2000us = 37 rpm。
// 相機軸負載輕，但第一次測試先慢一點，看清楚方向對不對再加速。
static uint32_t cruiseHalfUs = 800;

static const uint32_t RAMP_START_US = 2000;
static const uint32_t RAMP_STEPS    = 200;
static const uint32_t MAX_TRAVEL    = 200000UL;
static const uint32_t REPORT_STEPS  = 400;   // 每幾步印一次進度

static ArduinoGpio     enablePin(EN);
static ArduinoInputPin xLimitPin(X_LIMIT),     yLimitPin(Y_LIMIT);
static ArduinoInputPin xFarPin(X_LIMIT_FAR),   yFarPin(Y_LIMIT_FAR);
static ArduinoInputPin startPin(START_SW);

static Button xLimit(xLimitPin, 5, LIMIT_ACTIVE_LOW);
static Button yLimit(yLimitPin, 5, LIMIT_ACTIVE_LOW);
static Button xFar(xFarPin, 5, LIMIT_ACTIVE_LOW);
static Button yFar(yFarPin, 5, LIMIT_ACTIVE_LOW);
static Button startSwitch(startPin, 30, START_SW_ACTIVE_LOW);

static A4988Driver xDrv(X_STP, X_DIR);
static A4988Driver yDrv(Y_STP, Y_DIR);
static Stepper     xMotor(xDrv, CAM_PPR, cruiseHalfUs);
static Stepper     yMotor(yDrv, CAM_PPR, cruiseHalfUs);

static bool     testingX = true;    // 目前測哪一軸
static bool     towardHome = false; // 目前往哪個方向走
static bool     running = false;
static bool     watchOnly = false;  // s 指令：只看限位，不動馬達
static uint32_t stepsAtStart = 0;
static uint32_t lastReported = 0;
static uint32_t laps = 0;

static Stepper& motor()     { return testingX ? xMotor : yMotor; }
static Button&  homeLimit() { return testingX ? xLimit : yLimit; }
static Button&  farLimit()  { return testingX ? xFar   : yFar;   }
static bool     towardHomeDir() { return testingX ? X_TOWARD_HOME : Y_TOWARD_HOME; }

// 往某個方向前，先確認那一端沒有被壓著 —— 壓著還推就是撞機。
static bool canMove(bool toHome) {
    return toHome ? !homeLimit().isTriggered() : !farLimit().isTriggered();
}

static void beginRun(bool toHome) {
    if (!canMove(toHome)) {
        Serial.print(F("!! "));
        Serial.print(toHome ? F("原點") : F("遠端"));
        Serial.println(F("側限位已被壓住，不能往那邊走 —— 請手動推離再試"));
        motor().stop();
        running = false;
        return;
    }
    towardHome   = toHome;
    stepsAtStart = motor().stepsTaken();
    lastReported = 0;
    motor().setHalfPeriodUs(cruiseHalfUs);
    // toward_ 的布林值就是「朝原點」的那個方向，反向就取 !
    motor().moveUntilSignal(MAX_TRAVEL,
                            toHome ? towardHomeDir() : !towardHomeDir());
    Serial.print(F(">> "));
    Serial.print(testingX ? F("X") : F("Y"));
    Serial.print(F(" 軸往"));
    Serial.print(toHome ? F("原點") : F("遠端"));
    Serial.print(F("  巡航 "));
    Serial.print(cruiseHalfUs);
    Serial.print(F("us = 約 "));
    Serial.print(60000000UL / (2UL * cruiseHalfUs * CAM_PPR));
    Serial.println(F(" rpm"));
}

static void printLimits() {
    Serial.print(F("  D9  X原點 = "));
    Serial.print(xLimit.isTriggered() ? F("[壓下]") : F("[放開]"));
    Serial.print(F("   D12 X遠端 = "));
    Serial.println(xFar.isTriggered() ? F("[壓下]") : F("[放開]"));
    Serial.print(F("  D10 Y原點 = "));
    Serial.print(yLimit.isTriggered() ? F("[壓下]") : F("[放開]"));
    Serial.print(F("   D13 Y遠端 = "));
    Serial.println(yFar.isTriggered() ? F("[壓下]") : F("[放開]"));
    Serial.print(F("  A3  開關  = "));
    Serial.println(startSwitch.isDown() ? F("[按下]") : F("[放開]"));
}

static void printStatus() {
    Serial.println(F("--- 狀態 ---"));
    Serial.print(F("  測試軸 = "));
    Serial.println(testingX ? F("X (D2/D5)") : F("Y (D3/D6)"));
    Serial.print(F("  巡航半週期 = ")); Serial.print(cruiseHalfUs);
    Serial.print(F("us  約 "));
    Serial.print(60000000UL / (2UL * cruiseHalfUs * CAM_PPR));
    Serial.println(F(" rpm"));
    Serial.print(F("  PPR = ")); Serial.println(CAM_PPR);
    printLimits();
    Serial.println(F("------------"));
}

void setup() {
    Serial.begin(115200);
    while (!Serial) { ; }
    delay(200);

    enablePin.init();
    xLimitPin.init(); yLimitPin.init();
    xFarPin.init();   yFarPin.init();
    startPin.init();
    xDrv.init(); yDrv.init();

    xMotor.setRamp(RAMP_START_US, RAMP_STEPS);
    yMotor.setRamp(RAMP_START_US, RAMP_STEPS);
    enablePin.write(false);          // LOW = 啟用 A4988

    Serial.println();
    Serial.println(F("=== 相機 X/Y 軸測試 ==="));
    Serial.println();
    Serial.println(F("先打 s，用手逐顆壓四個限位，確認每顆都有反應。"));
    Serial.println(F("確認無誤後按 A3，該軸會在兩端之間來回走。"));
    Serial.println(F("A3 放開立刻停。第一次請手扶著、隨時準備斷電。"));
    Serial.println();
    Serial.println(F("指令： x/y 切換軸   數字 改速度(30-5000)   s 只看限位   ? 狀態"));
    Serial.println();
    printStatus();
}

void loop() {
    const uint32_t nowMs = millis();
    const uint32_t nowUs = micros();

    xLimit.tick(nowMs); yLimit.tick(nowMs);
    xFar.tick(nowMs);   yFar.tick(nowMs);
    startSwitch.tick(nowMs);
    xMotor.tick(nowUs); yMotor.tick(nowUs);

    // ── 序列埠指令 ──────────────────────────────────
    if (Serial.available()) {
        String cmd = Serial.readStringUntil('\n');
        cmd.trim();
        if (cmd == "?") {
            printStatus();
        } else if (cmd == "s") {
            watchOnly = !watchOnly;
            Serial.println(watchOnly ? F(">> 限位監看模式（馬達不動）")
                                     : F(">> 離開監看模式"));
            printLimits();
        } else if (cmd == "x" || cmd == "X") {
            if (running) { motor().stop(); running = false; }
            testingX = true;
            Serial.println(F(">> 切換到 X 軸"));
        } else if (cmd == "y" || cmd == "Y") {
            if (running) { motor().stop(); running = false; }
            testingX = false;
            Serial.println(F(">> 切換到 Y 軸"));
        } else if (cmd.length() > 0) {
            long v = cmd.toInt();
            if (v >= 30 && v <= 5000) {
                cruiseHalfUs = (uint32_t)v;
                xMotor.setHalfPeriodUs(cruiseHalfUs);
                yMotor.setHalfPeriodUs(cruiseHalfUs);
                Serial.print(F(">> 巡航改為 "));
                Serial.print(cruiseHalfUs);
                Serial.print(F("us = 約 "));
                Serial.print(60000000UL / (2UL * cruiseHalfUs * CAM_PPR));
                Serial.println(F(" rpm"));
            } else {
                Serial.println(F(">> 請輸入 30-5000，或 x / y / s / ?"));
            }
        }
    }

    // ── 監看模式：限位一有變化就印，不動馬達 ─────────
    if (watchOnly) {
        static bool prev[4] = { false, false, false, false };
        bool now[4] = { xLimit.isTriggered(), xFar.isTriggered(),
                        yLimit.isTriggered(), yFar.isTriggered() };
        for (uint8_t i = 0; i < 4; ++i) {
            if (now[i] != prev[i]) {
                prev[i] = now[i];
                static const char* const names[4] =
                    { "D9  X原點", "D12 X遠端", "D10 Y原點", "D13 Y遠端" };
                Serial.print(F("   "));
                Serial.print(names[i]);
                Serial.println(now[i] ? F("  → [壓下]") : F("  → [放開]"));
            }
        }
        if (running) { motor().stop(); running = false; }
        return;
    }

    const bool swDown = startSwitch.isDown();

    if (!swDown) {
        if (running) {
            motor().stop();
            running = false;
            Serial.print(F(">> A3 放開，停止。這趟走了 "));
            Serial.print(motor().stepsTaken() - stepsAtStart);
            Serial.println(F(" 步"));
            Serial.println();
        }
        return;
    }

    if (!running) {
        running = true;
        laps = 0;
        Serial.println();
        Serial.print(F(">> A3 按下，開始測 "));
        Serial.print(testingX ? F("X") : F("Y"));
        Serial.println(F(" 軸"));
        // 已經壓在原點側就先往遠端走，否則往原點走
        beginRun(!homeLimit().isTriggered());
        return;
    }

    // ── 進度回報 ────────────────────────────────────
    const uint32_t travelled = motor().stepsTaken() - stepsAtStart;
    if (travelled - lastReported >= REPORT_STEPS) {
        lastReported = travelled;
        Serial.print(F("   已走 "));
        Serial.print(travelled);
        Serial.print(F(" 步 ("));
        Serial.print(travelled / CAM_PPR);
        Serial.println(F(" 圈)"));
    }

    // ── 碰限位折返 ──────────────────────────────────
    if (towardHome && homeLimit().isTriggered()) {
        motor().stop();
        Serial.print(F("   碰到原點側限位，共 "));
        Serial.print(travelled);
        Serial.println(F(" 步，折返"));
        beginRun(false);
        return;
    }
    if (!towardHome && farLimit().isTriggered()) {
        motor().stop();
        ++laps;
        Serial.print(F("   碰到遠端限位，共 "));
        Serial.print(travelled);
        Serial.print(F(" 步 ── 完成第 "));
        Serial.print(laps);
        Serial.println(F(" 趟來回"));
        beginRun(true);
        return;
    }

    if (!motor().isMoving()) {
        motor().stop();
        running = false;
        Serial.println(F("!! 走完上限步數仍未碰到限位 —— 檢查接線或方向是否相反"));
    }
}
