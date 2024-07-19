#include <cstdint>

#pragma once

class DriverInterface {
protected:
    DriverInterface() = default;

public:
    virtual ~DriverInterface() = default;
    virtual void flashLED() = 0;
    virtual void wait(uint32_t timeout_ms) = 0;
};
