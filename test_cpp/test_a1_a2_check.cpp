/*
 * A1 樣本偵測 / A2 拍照位置 —— 極性確認
 *
 * 用法：複製成 src/main.cpp 燒錄。正式韌體備份在 main_reg2.cpp。
 *
 * 先確認極性再改正式韌體：判斷相反的話「有樣本」會被讀成「沒樣本」，
 * 流程整個倒過來，而且不會報錯，很難查。
 *
 * 兩線 NC 接法（與其他六顆限位相同）：
 *     開關 C 腳  -> GND
 *     開關 NC 腳 -> A1 或 A2
 *   放開 -> 導通 -> LOW  -> 判定 [正常]
 *   壓下 -> 斷開 -> HIGH -> 判定 [觸發]
 *
 * 馬達全程不通電（EN 拉 HIGH）。
 */
#include <Arduino.h>
#include "hal/ArduinoGpio.h"
#include "hal/ArduinoInputPin.h"
#include "Button.h"

static const uint8_t EN             = 8;
static const uint8_t SAMPLE_PRESENT = A1;   // Hold 接點
static const uint8_t IR_SENSOR      = A2;   // Resume 接點

// 兩線 NC 接法：斷開（HIGH）算觸發。若實測相反，就是接到 NO 腳了。
static const bool ACTIVE_LOW = false;

static ArduinoGpio     enablePin(EN);
static ArduinoInputPin samplePin(SAMPLE_PRESENT), irPin(IR_SENSOR);
static Button sampleSw(samplePin, 20, ACTIVE_LOW);
static Button irSw(irPin, 20, ACTIVE_LOW);

static int  lastRawS = -1, lastRawI = -1;
static bool lastTrigS = false, lastTrigI = false;
static uint32_t lastReport = 0;

static void printRow(const __FlashStringHelper* tag) {
    Serial.print(tag);
    Serial.print(F("  A1 樣本 原始="));
    Serial.print(digitalRead(SAMPLE_PRESENT) ? F("HIGH") : F("LOW "));
    Serial.print(F(" 判定="));
    Serial.print(sampleSw.isTriggered() ? F("[有樣本]") : F("[沒樣本]"));
    Serial.print(F("    A2 到位 原始="));
    Serial.print(digitalRead(IR_SENSOR) ? F("HIGH") : F("LOW "));
    Serial.print(F(" 判定="));
    Serial.println(irSw.isTriggered() ? F("[已到位]") : F("[未到位]"));
}

void setup() {
    Serial.begin(115200);
    while (!Serial) { ; }
    delay(200);

    enablePin.init();
    enablePin.write(true);            // HIGH = 關閉驅動器，馬達不動
    samplePin.init(); irPin.init();

    Serial.println();
    Serial.println(F("=== A1 / A2 極性確認（馬達不通電）==="));
    Serial.println();
    Serial.println(F("NC 接法應該是："));
    Serial.println(F("  放開 -> LOW  -> [沒樣本] / [未到位]"));
    Serial.println(F("  壓下 -> HIGH -> [有樣本] / [已到位]"));
    Serial.println();
    Serial.println(F("逐顆用手壓，對照上面："));
    Serial.println(F("  完全沒反應 -> 線沒接上或插錯孔"));
    Serial.println(F("  剛好相反   -> 接到 NO 腳了，改接 NC"));
    Serial.println(F("  數值亂跳   -> 接觸不良"));
    Serial.println();
    delay(300);
    printRow(F("開機 "));
}

void loop() {
    const uint32_t nowMs = millis();
    sampleSw.tick(nowMs);
    irSw.tick(nowMs);

    const int  rs = digitalRead(SAMPLE_PRESENT);
    const int  ri = digitalRead(IR_SENSOR);
    const bool ts = sampleSw.isTriggered();
    const bool ti = irSw.isTriggered();

    if (rs != lastRawS || ri != lastRawI || ts != lastTrigS || ti != lastTrigI) {
        lastRawS = rs; lastRawI = ri; lastTrigS = ts; lastTrigI = ti;
        printRow(F("變化 "));
    }

    if (nowMs - lastReport >= 2000) {
        lastReport = nowMs;
        printRow(F("定時 "));
    }
}
