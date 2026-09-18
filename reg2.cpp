// === 效能量測韌體 ===========================================================
// 量 loop() 一輪實際要多久 —— 這決定了脈波頻率的天花板。
// 不驅動馬達：DIR/PUL 照常送訊號，但你可以不開 DM542 電源。
#include <Arduino.h>
#include "hal/ArduinoGpio.h"
#include "hal/ArduinoInputPin.h"
#include "hal/Dm542Driver.h"
#include "Blinker.h"
#include "Button.h"
#include "Stepper.h"

static const uint8_t  PIN_PUL = 3, PIN_DIR = 2, PIN_BUTTON = 4, PIN_LIMIT = 5;
static const uint32_t PULSE_PER_REV = 1600;

static ArduinoGpio     led(LED_BUILTIN);
static Blinker         blinker(led, 500);
static ArduinoInputPin buttonPin(PIN_BUTTON);
static Button          button(buttonPin);
static ArduinoInputPin limitPin(PIN_LIMIT);
static Button          limitBottom(limitPin, 20, false);
static Dm542Driver     driver(PIN_PUL, PIN_DIR);
static Stepper         stepper(driver, PULSE_PER_REV, 1);   // 巡航設 1us = 要求極限

static uint32_t loopCount = 0, worstLoopUs = 0, lastReport = 0;
static uint32_t stepsAtReport = 0;

void setup() {
    Serial.begin(9600);
    led.init(); buttonPin.init(); limitPin.init(); driver.init();
    // 不設斜坡，直接要求全速 —— 量的是純粹的天花板
    Serial.println();
    Serial.println(F("=== loop speed benchmark ==="));
    Serial.println(F("cruise set to 1us (impossible) to find the real ceiling"));
    Serial.println();
    delay(500);
    stepper.moveRevolutions(1000, true);   // 排一大段，讓它一直跑
}

void loop() {
    uint32_t t0 = micros();

    blinker.tick(millis());
    button.tick(millis());
    limitBottom.tick(millis());
    stepper.tick(micros());

    uint32_t dt = micros() - t0;
    if (dt > worstLoopUs) worstLoopUs = dt;
    ++loopCount;

    if (millis() - lastReport >= 2000) {
        uint32_t steps = stepper.stepsTaken();
        uint32_t dSteps = steps - stepsAtReport;
        // 2 秒內走了 dSteps 步
        uint32_t pps = dSteps / 2;                       // pulses per second
        uint32_t rpm = (uint32_t)(((uint64_t)pps * 60) / PULSE_PER_REV);

        Serial.print(F("loops/s = "));   Serial.print(loopCount / 2);
        Serial.print(F("   worst loop = ")); Serial.print(worstLoopUs); Serial.print(F("us"));
        Serial.print(F("   pulses/s = ")); Serial.print(pps);
        Serial.print(F("   -> "));       Serial.print(rpm); Serial.println(F(" rpm"));

        loopCount = 0; worstLoopUs = 0; stepsAtReport = steps;
        lastReport = millis();
        if (!stepper.isMoving()) stepper.moveRevolutions(1000, true);
    }
}
