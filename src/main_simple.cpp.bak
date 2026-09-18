#include <Arduino.h>
#include "hal/ArduinoGpio.h"
#include "hal/ArduinoInputPin.h"
#include "hal/Dm542Driver.h"
#include "Blinker.h"
#include "Button.h"
#include "Stepper.h"

// main 應該薄到只剩「接線」：建立實體、注入依賴、驅動。

// --- 硬體設定 ---------------------------------------------------------
// 馬達 57HS56 (NEMA23, 1.8deg = 200 步/轉)
// DM542 DIP: SW1-3 = off/on/off (3.31A)、SW4 = off、SW5-8 = off/off/on/on
static const uint8_t  PIN_PUL = 3;
static const uint8_t  PIN_DIR = 2;
static const uint32_t PULSE_PER_REV = 1600;   // 200 步 x 8 細分

// 自鎖式開關 (latching)：壓一下持續導通，再壓一下斷開 —— 開關本身記住狀態。
// 一腳接 D4，另一腳接 GND。不需要外接電阻 (用內部上拉)。
static const uint8_t  PIN_BUTTON = 4;

// 底部極限開關 (常閉 NC)：
//   開關 C  -> Arduino GND
//   開關 NC -> Arduino D5
//   開關 NO -> 空著不接
// 平時 C-NC 導通接地讀到 LOW；滑塊壓到開關時斷開讀到 HIGH。
// 線鬆脫或開關損壞時同樣讀到 HIGH，會被判定成觸發 —— 失效安全。
static const uint8_t  PIN_LIMIT_BOTTOM = 5;

// 滑塊往底部 (極限開關那一側) 是哪個方向。
// 還沒確認，先手動點動看哪邊對，錯了就把這行改成 true。
static const bool     HOME_DIR = false;

// 半週期 -> 轉速: rpm = 60e6 / (2 * HALF_PERIOD_US * PULSE_PER_REV)
// 50us -> 約 375 rpm。這是巡航速度，起步和收尾會自動放慢 (見 RAMP_*)。
static const uint32_t HALF_PERIOD_US = 50;

// 加速斜坡：起步 600us (約 31 rpm)，400 步之內收斂到巡航速度；
// 接近終點時對稱地減速回來。沒有斜坡的話 50us 從靜止直接啟動，
// 空載勉強轉得動，一接負載就會失步或堵轉。
static const uint32_t RAMP_START_US = 600;
static const uint32_t RAMP_STEPS    = 400;

// 每次動作轉幾圈。要精確角度的話改用 moveSteps()，
// 一圈 = PULSE_PER_REV 步，90 度就是 400 步。
static const uint32_t REVS_PER_MOVE = 5;

// 來回之間的停頓 (毫秒)。這是等待，不是 delay() ——
// 期間 loop() 照常跑，LED 也照常閃。
static const uint32_t PAUSE_MS = 1500;

// LED 用閃爍速度表示狀態：
//   慢閃 (500ms) = 正常
//   快閃 (100ms) = 極限開關觸發，故障閂鎖中
static const uint32_t BLINK_NORMAL_MS = 500;
static const uint32_t BLINK_FAULT_MS  = 100;

static ArduinoGpio     led(LED_BUILTIN);
static Blinker         blinker(led, BLINK_NORMAL_MS);

static ArduinoInputPin buttonPin(PIN_BUTTON);
static Button          button(buttonPin);

// activeLow = false：NC 接法，接點「斷開」才算觸發。
static ArduinoInputPin limitPin(PIN_LIMIT_BOTTOM);
static Button          limitBottom(limitPin, 20, false);

static Dm542Driver     driver(PIN_PUL, PIN_DIR);
static Stepper         stepper(driver, PULSE_PER_REV, HALF_PERIOD_US);

// --- 動作排程 ---------------------------------------------------------
// 開關導通 = 持續來回運作，斷開 = 走完目前這一趟才停。
// 因為是自鎖式開關，狀態由開關自己記住，程式只要讀當下電平即可 ——
// 不需要自己維護「現在是開還關」的旗標。
//
// 斷開時不急停：會把目前這一趟走完才真的停下來，所以馬達永遠停在
// 一趟的終點，位置是可預期的。
static uint32_t pauseStartedMs = 0;
static bool     paused         = false;
static bool     nextForward    = true;

// 極限開關跳脫後閂鎖住，不會因為滑塊退開就自動恢復運轉 ——
// 觸發過就必須人為介入 (把自鎖開關切到斷開再切回導通) 才能重新啟動。
// 沒有這道閂鎖的話，急停後開關一鬆開機器就自己又動起來，很危險。
static bool     faulted = false;

void setup() {
    led.init();
    buttonPin.init();
    limitPin.init();
    driver.init();
    stepper.setRamp(RAMP_START_US, RAMP_STEPS);
    delay(500);                     // 等驅動器上電穩定
}

void loop() {
    uint32_t nowMs = millis();

    blinker.tick(nowMs);            // LED 照常閃，證明 loop 沒有被卡住
    button.tick(nowMs);
    limitBottom.tick(nowMs);
    stepper.tick(micros());         // 脈波要微秒解析度

    // --- 安全：極限開關 ---------------------------------------------
    // 這裡是急停，不是「跑完這趟才停」—— 滑塊已經到底了，多走一步都是
    // 在頂機構。Stepper::stop() 會把脈波收在閒置準位並丟棄剩餘步數。
    if (limitBottom.isTriggered() && !faulted) {
        stepper.stop();
        faulted = true;
        paused  = true;
        blinker.setPeriod(BLINK_FAULT_MS);   // 快閃 = 出事了
    }

    // 解除故障：把自鎖開關切到斷開，且滑塊已離開極限開關。
    // 要求兩個條件同時成立，避免開關還壓著就被清掉。
    if (faulted && !button.isDown() && !limitBottom.isTriggered()) {
        faulted = false;
        blinker.setPeriod(BLINK_NORMAL_MS);
    }

    // 剛切到導通：把停頓視為已經結束，下一輪就出發，不用再等 PAUSE_MS。
    if (button.wasPressed()) {
        paused         = true;
        pauseStartedMs = nowMs - PAUSE_MS;
    }

    if (!stepper.isMoving()) {
        if (!paused) {
            paused         = true;
            pauseStartedMs = nowMs;
        } else if (!faulted && button.isDown() && nowMs - pauseStartedMs >= PAUSE_MS) {
            paused      = false;
            nextForward = !nextForward;
            stepper.moveRevolutions(REVS_PER_MOVE, nextForward);
        }
    }
}
