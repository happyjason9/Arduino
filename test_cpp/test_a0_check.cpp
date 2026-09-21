/*
 * A0 底部限位 單腳診斷
 *
 * 用法：複製成 src/main.cpp 燒錄。正式韌體備份在 main_reg2.cpp。
 *
 * 為什麼要單獨測這支：換了新開關、韌體也加了全域保護，輸送台卻還是撞上去。
 * 那表示「程式沒看到觸發」，而不是「看到了但沒停」。可能原因有三個，
 * 這支程式一次把三個都分開來看：
 *
 *   1. 接線錯（線沒接到 A0、或接到別的孔）      -> 壓開關時完全沒反應
 *   2. 極性相反（接了 NO 而不是 NC）            -> 平常就顯示觸發，壓下才解除
 *   3. 開關本身壞了 / 接觸不良                  -> 讀值亂跳
 *
 * 馬達全程不通電（EN 拉 HIGH），純粹讀腳位，撞不到東西。
 *
 * 顯示：A0 與 D11 的原始電位、經去彈跳後的判定，只要有變化就印一行，
 *       另外每兩秒印一次目前狀態，方便確認「沒反應」不是因為程式當掉。
 */
#include <Arduino.h>
#include "hal/ArduinoGpio.h"
#include "hal/ArduinoInputPin.h"
#include "Button.h"

static const uint8_t EN          = 8;
static const uint8_t CONV_BOTTOM = A0;   // Abort 接點
static const uint8_t CONV_TOP    = 11;   // END STOPS Z+

// 與正式韌體相同：NC 接法，斷開（HIGH）算觸發。
static const bool ACTIVE_LOW = false;

static ArduinoGpio     enablePin(EN);
static ArduinoInputPin bottomPin(CONV_BOTTOM), topPin(CONV_TOP);
static Button bottom(bottomPin, 5, ACTIVE_LOW);
static Button top(topPin, 5, ACTIVE_LOW);

static int  lastRawBottom = -1, lastRawTop = -1;
static bool lastTrigBottom = false, lastTrigTop = false;
static uint32_t lastReport = 0;

static void printRow(const __FlashStringHelper* tag) {
    Serial.print(tag);
    Serial.print(F("  A0 原始="));
    Serial.print(digitalRead(CONV_BOTTOM) ? F("HIGH") : F("LOW "));
    Serial.print(F(" 判定="));
    Serial.print(bottom.isTriggered() ? F("[觸發]") : F("[正常]"));
    Serial.print(F("    D11 原始="));
    Serial.print(digitalRead(CONV_TOP) ? F("HIGH") : F("LOW "));
    Serial.print(F(" 判定="));
    Serial.println(top.isTriggered() ? F("[觸發]") : F("[正常]"));
}

void setup() {
    Serial.begin(115200);
    while (!Serial) { ; }
    delay(200);

    enablePin.init();
    enablePin.write(true);          // HIGH = 關閉驅動器，馬達完全不動
    bottomPin.init(); topPin.init();

    Serial.println();
    Serial.println(F("=== A0 / D11 限位診斷（馬達不通電）==="));
    Serial.println();
    Serial.println(F("NC 接法的正確表現："));
    Serial.println(F("  放開 -> 原始 LOW  判定 [正常]"));
    Serial.println(F("  壓下 -> 原始 HIGH 判定 [觸發]"));
    Serial.println();
    Serial.println(F("用手壓 A0 那顆開關，看有沒有照上面變化："));
    Serial.println(F("  完全沒反應      -> 線沒接到 A0，或接錯孔"));
    Serial.println(F("  剛好相反        -> 接成 NO 了，要改接 NC 腳"));
    Serial.println(F("  數值亂跳        -> 開關壞了或接觸不良"));
    Serial.println();
    delay(300);
    printRow(F("開機 "));
}

void loop() {
    const uint32_t nowMs = millis();
    bottom.tick(nowMs);
    top.tick(nowMs);

    const int  rb = digitalRead(CONV_BOTTOM);
    const int  rt = digitalRead(CONV_TOP);
    const bool tb = bottom.isTriggered();
    const bool tt = top.isTriggered();

    if (rb != lastRawBottom || rt != lastRawTop ||
        tb != lastTrigBottom || tt != lastTrigTop) {
        lastRawBottom = rb; lastRawTop = rt;
        lastTrigBottom = tb; lastTrigTop = tt;
        printRow(F("變化 "));
    }

    if (nowMs - lastReport >= 2000) {
        lastReport = nowMs;
        printRow(F("定時 "));
    }
}
