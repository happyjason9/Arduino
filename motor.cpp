// === XY 雙軸步進馬達控制韌體 ===============================================
// 由 Python 端透過序列埠送出文字指令，本韌體解析後驅動 X/Y 軸步進馬達。
// 通訊協議：Python 送一行指令 -> Arduino 執行 -> 回覆 "OK" -> 才送下一筆。
// 這是 motor.ino 的 C++ (PlatformIO) 版本。
#include <Arduino.h>

static const uint8_t PIN_X_DIR = 5;   // X 軸方向控制腳
static const uint8_t PIN_X_STP = 2;   // X 軸步進脈衝腳
static const uint8_t PIN_Y_DIR = 6;   // Y 軸方向控制腳
static const uint8_t PIN_Y_STP = 3;   // Y 軸步進脈衝腳
static const uint8_t PIN_EN    = 8;   // 驅動器致能腳 (LOW = 致能)

static const uint16_t STEPS_PER_COL = 1360;   // X 軸移動一欄的步數
static const uint16_t STEPS_PER_ROW = 1040;   // Y 軸移動一列的步數
static const uint16_t STEP_DELAY_US = 800;    // 每個脈衝半週期的長度

// 逐步驅動單一軸：拉高 -> 等待 -> 拉低 -> 等待，重複 steps 次。
static void stepMotor(uint8_t stpPin, uint32_t steps, uint16_t delayUs) {
    for (uint32_t i = 0; i < steps; ++i) {
        digitalWrite(stpPin, HIGH);
        delayMicroseconds(delayUs);
        digitalWrite(stpPin, LOW);
        delayMicroseconds(delayUs);
    }
}

// 同時驅動兩軸：以較大的步數為迴圈上限，步數較少的軸提前停止觸發，
// 讓兩軸接近同時到位。（保留給需要對角移動時使用，目前 loop() 未呼叫）
__attribute__((unused))
static void stepBoth(uint32_t xSteps, uint32_t ySteps, uint16_t delayUs) {
    const uint32_t maxSteps = max(xSteps, ySteps);
    for (uint32_t i = 0; i < maxSteps; ++i) {
        if (i < xSteps) digitalWrite(PIN_X_STP, HIGH);
        if (i < ySteps) digitalWrite(PIN_Y_STP, HIGH);
        delayMicroseconds(delayUs);
        if (i < xSteps) digitalWrite(PIN_X_STP, LOW);
        if (i < ySteps) digitalWrite(PIN_Y_STP, LOW);
        delayMicroseconds(delayUs);
    }
}

// 設定方向後移動指定步數。
// signedSteps = false：負數視為無效，直接忽略不動作（一般移動指令）。
// signedSteps = true ：負數代表反向，取絕對值後往反方向走（HOME 指令用）。
static void moveAxis(uint8_t dirPin, uint8_t stpPin, int32_t steps,
                     bool dirForward, bool signedSteps = false) {
    if (steps < 0) {
        if (!signedSteps) return;              // 負數忽略
        dirForward = !dirForward;              // 負數反向
        steps = -steps;
    }
    digitalWrite(dirPin, dirForward ? HIGH : LOW);
    stepMotor(stpPin, (uint32_t)steps, STEP_DELAY_US);
}

void setup() {
    pinMode(PIN_X_DIR, OUTPUT);
    pinMode(PIN_X_STP, OUTPUT);
    pinMode(PIN_Y_DIR, OUTPUT);
    pinMode(PIN_Y_STP, OUTPUT);
    pinMode(PIN_EN,    OUTPUT);
    digitalWrite(PIN_EN, LOW);        // 致能驅動器
    Serial.begin(115200);
    Serial.println(F("READY"));       // 告知 Python 端已就緒
}

void loop() {
    if (!Serial.available()) return;

    String cmd = Serial.readStringUntil('\n');
    cmd.trim();

    if (cmd.startsWith(F("X+:"))) {
        // X 軸正向移動指定步數
        moveAxis(PIN_X_DIR, PIN_X_STP, cmd.substring(3).toInt(), true);

    } else if (cmd.startsWith(F("X-:"))) {
        // X 軸反向移動指定步數
        moveAxis(PIN_X_DIR, PIN_X_STP, cmd.substring(3).toInt(), false);

    } else if (cmd == F("X+")) {
        // X 軸正向移動一欄
        moveAxis(PIN_X_DIR, PIN_X_STP, STEPS_PER_COL, true);

    } else if (cmd == F("X-")) {
        // X 軸反向移動一欄
        moveAxis(PIN_X_DIR, PIN_X_STP, STEPS_PER_COL, false);

    } else if (cmd.startsWith(F("Y+:"))) {
        // Y 軸移動指定步數（Y+ 對應 DIR = LOW）
        moveAxis(PIN_Y_DIR, PIN_Y_STP, cmd.substring(3).toInt(), false);

    } else if (cmd == F("Y+")) {
        // Y 軸移動一列
        moveAxis(PIN_Y_DIR, PIN_Y_STP, STEPS_PER_ROW, false);

    } else if (cmd == F("HOME")) {
        // 保留指令：X/Y 同時回原點的方向與步數由 Python 端算好，
        // 再分別以 XHOME:/YHOME: 送來，這裡不處理。

    } else if (cmd.startsWith(F("XHOME:"))) {
        // X 軸回原點，帶號步數決定方向（負數 = 反向）
        moveAxis(PIN_X_DIR, PIN_X_STP, cmd.substring(6).toInt(), true, true);

    } else if (cmd.startsWith(F("YHOME:"))) {
        // Y 軸回原點（固定 DIR = HIGH，與 Y+ 相反方向）
        moveAxis(PIN_Y_DIR, PIN_Y_STP, cmd.substring(6).toInt(), true);
    }

    Serial.println(F("OK"));   // 回報完成，Python 收到後才送下一筆
}
