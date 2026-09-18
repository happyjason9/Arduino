#pragma once
#include <stdint.h>

// 拍照請求的抽象：domain 層只知道「請 PC 拍一張、問它好了沒」，
// 不知道底下是 Serial、是 mock、還是別的通訊方式。
class ICapture {
public:
    virtual ~ICapture() = default;
    virtual void request(uint16_t col, uint16_t row) = 0;
    virtual bool isDone() const = 0;      // PC 回了 OK
    virtual bool isFailed() const = 0;    // 逾時或回了錯誤
};
