/*
 * 輸送台 診斷：找出「轉幾圈會卡」的規律
 *
 * 正式韌體備份在專案根目錄 main_reg.cpp。
 *
 * 目的：卡住的位置是固定的還是隨機的 —— 這兩種原因完全不同。
 *   固定位置卡 → 機械問題（絲桿彎、異物、導軌不平行、聯軸器沒對正）
 *   隨機位置卡 → 電氣問題（電流不足、共振、電源功率不夠）
 *
 * 行為：A3 按下後往一個方向連續轉，每轉一圈印一次累計圈數與步數。
 *       碰到限位會自動折返，方向不對也不會硬撞。
 *       卡住時記下畫面上的圈數，放開 A3 停止，再按一次重跑。
 *       跑三四趟，比對每次卡住的圈數是否相同。
 *
 * 指令（序列埠 115200）：
 *   數字     改巡航半週期，例如打 150 就是 150us（越小越快）
 *   ?        印出目前設定與狀態
 *
 * 序列埠 115200。
 */
#include <Arduino.h>
#include "hal/ArduinoGpio.h"
#include "hal/ArduinoInputPin.h"
#include "hal/Dm542Driver.h"
#include "Button.h"
#include "Stepper.h"

static const uint8_t EN       = 8;
static const uint8_t CONV_PUL = 4, CONV_DIR = 7;
static const uint8_t CONV_TOP    = 11;
static const uint8_t CONV_BOTTOM = A0;
static const uint8_t START_SW    = A3;

static const bool CONV_TOP_ACTIVE_LOW    = false;
static const bool CONV_BOTTOM_ACTIVE_LOW = false;
static const bool START_SW_ACTIVE_LOW    = true;

static const bool CONV_DOWN = true;
static const bool CONV_UP   = false;

// 2026-09-16 實測：DIP 由 1/8 微步（1600）改成 1/4 微步（800）後扭力足夠，
// 原本推不動的阻力點都過得去了。微步越粗單步扭力越大。
static const uint32_t CONV_PPR = 800;

// 轉速 = 60e6 / (2 * 半週期us * PPR)。800 PPR 下：
//   50us = 750 rpm（過快，會失步）   100us = 375 rpm
//   200us = 187 rpm                  400us = 93 rpm
//   800us = 47 rpm                   1600us = 23 rpm
// 預設取 400us（93 rpm）：遠離 5-15 rpm 共振帶，又還在扭力接近滿載的低速區。
static uint32_t cruiseHalfUs  = 400;    // 可用序列埠即時調整

// 斜坡起步值要比巡航慢才有意義（setRamp 要求 start >= cruise）。
static const uint32_t RAMP_START_US = 1200;
static const uint32_t RAMP_STEPS    = 200;
static const uint32_t MAX_TRAVEL    = 400000UL;

static ArduinoGpio     enablePin(EN);
static ArduinoInputPin convTopPin(CONV_TOP), convBottomPin(CONV_BOTTOM);
static ArduinoInputPin startPin(START_SW);

static Button convTop(convTopPin, 5, CONV_TOP_ACTIVE_LOW);
static Button convBottom(convBottomPin, 5, CONV_BOTTOM_ACTIVE_LOW);
static Button startSwitch(startPin, 30, START_SW_ACTIVE_LOW);

static Dm542Driver convDrv(CONV_PUL, CONV_DIR);
static Stepper     conveyor(convDrv, CONV_PPR, cruiseHalfUs);

enum class Dir : uint8_t { Down, Up };
static Dir      dir = Dir::Down;
static bool     running = false;
static uint32_t stepsAtStart = 0;
static uint32_t lastReportedRev = 0;
static uint32_t laps = 0;

static void beginRun(Dir d) {
    dir = d;
    stepsAtStart    = conveyor.stepsTaken();
    lastReportedRev = 0;
    conveyor.setHalfPeriodUs(cruiseHalfUs);
    conveyor.moveUntilSignal(MAX_TRAVEL, (d == Dir::Down) ? CONV_DOWN : CONV_UP);
    Serial.print(F(">> 往"));
    Serial.print((d == Dir::Down) ? F("底部") : F("頂部"));
    Serial.print(F("  巡航 "));
    Serial.print(cruiseHalfUs);
    Serial.print(F("us = 約 "));
    Serial.print(60000000UL / (2UL * cruiseHalfUs * CONV_PPR));
    Serial.println(F(" rpm"));
}

static void printStatus() {
    Serial.println(F("--- 狀態 ---"));
    Serial.print(F("  巡航半週期 = ")); Serial.print(cruiseHalfUs);
    Serial.print(F("us  約 "));
    Serial.print(60000000UL / (2UL * cruiseHalfUs * CONV_PPR));
    Serial.println(F(" rpm"));
    Serial.print(F("  PPR = ")); Serial.println(CONV_PPR);
    Serial.print(F("  D11 頂部 = "));
    Serial.print(convTop.isTriggered() ? F("[觸發]") : F("[未觸發]"));
    Serial.print(F("   A0 底部 = "));
    Serial.print(convBottom.isTriggered() ? F("[觸發]") : F("[未觸發]"));
    Serial.print(F("   A3 = "));
    Serial.println(startSwitch.isDown() ? F("[按下]") : F("[放開]"));
    Serial.println(F("------------"));
}

void setup() {
    Serial.begin(115200);
    while (!Serial) { ; }
    delay(200);

    enablePin.init();
    convTopPin.init(); convBottomPin.init();
    startPin.init();
    convDrv.init();

    conveyor.setRamp(RAMP_START_US, RAMP_STEPS);
    enablePin.write(false);

    Serial.println();
    Serial.println(F("=== 輸送台診斷：找卡住的規律 ==="));
    Serial.println();
    Serial.println(F("A3 按下 → 連續轉，每圈印一行。碰限位自動折返。"));
    Serial.println(F("卡住時記下圈數，放開 A3 停止，再按一次重跑。"));
    Serial.println(F("跑三四趟，比對每次卡住的圈數是否相同："));
    Serial.println(F("  每次都卡在同樣圈數 → 機械問題"));
    Serial.println(F("  每次卡的位置不同   → 電氣問題（電流/共振/電源）"));
    Serial.println();
    Serial.println(F("打數字可改速度，例如 150 或 400。打 ? 看狀態。"));
    Serial.println();
    printStatus();
}

void loop() {
    const uint32_t nowMs = millis();
    const uint32_t nowUs = micros();

    convTop.tick(nowMs);
    convBottom.tick(nowMs);
    startSwitch.tick(nowMs);
    conveyor.tick(nowUs);

    // ── 序列埠指令 ──────────────────────────────────
    if (Serial.available()) {
        String cmd = Serial.readStringUntil('\n');
        cmd.trim();
        if (cmd == "?") {
            printStatus();
        } else if (cmd.length() > 0) {
            long v = cmd.toInt();
            if (v >= 30 && v <= 5000) {
                cruiseHalfUs = (uint32_t)v;
                conveyor.setHalfPeriodUs(cruiseHalfUs);
                Serial.print(F(">> 巡航改為 "));
                Serial.print(cruiseHalfUs);
                Serial.print(F("us = 約 "));
                Serial.print(60000000UL / (2UL * cruiseHalfUs * CONV_PPR));
                Serial.println(F(" rpm"));
            } else {
                Serial.println(F(">> 請輸入 30-5000 之間的數字，或 ?"));
            }
        }
    }

    const bool swDown = startSwitch.isDown();

    if (!swDown) {
        if (running) {
            conveyor.stop();
            running = false;
            uint32_t rev = (conveyor.stepsTaken() - stepsAtStart) / CONV_PPR;
            Serial.print(F(">> A3 放開，停止。這趟走了 "));
            Serial.print(rev);
            Serial.println(F(" 圈"));
            Serial.println();
        }
        return;
    }

    if (!running) {
        running = true;
        laps = 0;
        Serial.println();
        Serial.println(F(">> A3 按下，開始"));
        beginRun(convBottom.isTriggered() ? Dir::Up : Dir::Down);
        return;
    }

    // ── 每轉一圈印一行 ──────────────────────────────
    const uint32_t travelled = conveyor.stepsTaken() - stepsAtStart;
    const uint32_t rev = travelled / CONV_PPR;
    if (rev != lastReportedRev) {
        lastReportedRev = rev;
        Serial.print(F("   第 "));
        Serial.print(rev);
        Serial.print(F(" 圈   累計步數 "));
        Serial.println(travelled);
    }

    // ── 碰限位折返 ──────────────────────────────────
    if (dir == Dir::Down && convBottom.isTriggered()) {
        conveyor.stop();
        Serial.print(F("   碰到底部 A0，共 "));
        Serial.print(rev);
        Serial.println(F(" 圈，折返"));
        beginRun(Dir::Up);
        return;
    }
    if (dir == Dir::Up && convTop.isTriggered()) {
        conveyor.stop();
        ++laps;
        Serial.print(F("   碰到頂部 D11，共 "));
        Serial.print(rev);
        Serial.print(F(" 圈 ── 完成第 "));
        Serial.print(laps);
        Serial.println(F(" 趟來回"));
        beginRun(Dir::Down);
        return;
    }

    if (!conveyor.isMoving()) {
        conveyor.stop();
        running = false;
        Serial.println(F("!! 走完上限步數仍未碰到限位 —— 檢查接線或方向"));
    }
}
