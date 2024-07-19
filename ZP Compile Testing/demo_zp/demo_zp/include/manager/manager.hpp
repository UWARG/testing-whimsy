#include <cstdint>
#include "driver_interface.hpp"

#pragma once

class Manager {
private:
    DriverInterface *LEDDriver;
    uint8_t count;

    void flashOnce();
    void flashTwice(uint32_t timeout_ms);

public:
    Manager(DriverInterface *driverPtr);
    void runManager();
};
