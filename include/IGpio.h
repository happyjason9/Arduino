#pragma once

// GPIO 的抽象介面。
// domain 層只認識這個介面，不知道底下是 Arduino、是 mock、還是別的 MCU。
// 這就是「依賴反轉」：高階邏輯不依賴低階細節，兩者都依賴這個抽象。
class IGpio {
public:
    virtual ~IGpio() = default;
    virtual void init() = 0;
    virtual void write(bool high) = 0;
    virtual bool read() const = 0;
};
