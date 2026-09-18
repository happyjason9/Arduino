#pragma once
#include <Arduino.h>
#include "ICapture.h"

// HAL 層：透過 Serial 請 PC 端的 Python 拍照。
//
// 協定 (跟 system.ino 原本一致)：
//   Arduino -> PC :  CAPTURE:<row>,<col>\n
//   PC -> Arduino :  OK\n
//
// 非阻塞：request() 送出後立刻返回，之後每輪 loop 呼叫 poll() 收字元。
// 原本的阻塞版本會在這裡卡最多 10 秒，期間停止鈕完全沒反應。
class SerialCapture : public ICapture {
public:
    explicit SerialCapture(uint32_t timeoutMs = 10000) : timeout_(timeoutMs) {}

    void request(uint16_t col, uint16_t row) override {
        Serial.print(F("CAPTURE:"));
        Serial.print(row);
        Serial.print(',');
        Serial.println(col);
        sentAt_ = millis();
        len_ = 0;
        done_ = false;
        failed_ = false;
    }

    bool isDone()   const override { return done_; }
    bool isFailed() const override { return failed_; }

    // 每輪 loop 都要呼叫：收 Serial 字元、檢查逾時。
    void poll(uint32_t nowMs) {
        if (done_ || failed_) return;

        while (Serial.available()) {
            char c = Serial.read();
            if (c == '\n' || c == '\r') {
                if (len_ > 0) {
                    buf_[len_] = '\0';
                    if (strcmp(buf_, "OK") == 0) { done_ = true; return; }
                    len_ = 0;                    // 不是 OK，繼續等
                }
            } else if (len_ < sizeof(buf_) - 1) {
                buf_[len_++] = c;
            }
            // 超長的行直接丟掉多餘字元，避免緩衝區溢位
        }

        if (nowMs - sentAt_ >= timeout_) failed_ = true;
    }

private:
    uint32_t timeout_;
    uint32_t sentAt_ = 0;
    char     buf_[16];
    uint8_t  len_ = 0;
    bool     done_ = false;
    bool     failed_ = false;
};
