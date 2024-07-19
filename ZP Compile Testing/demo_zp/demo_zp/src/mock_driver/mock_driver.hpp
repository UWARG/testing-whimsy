#include <cstdint>
#include "driver_interface.hpp"
#include "gmock/gmock.h"

#pragma once

class MockDriver : public DriverInterface {
public:
    MOCK_METHOD(void, flashLED, (), (override));
    MOCK_METHOD(void, wait, (uint32_t timeout_ms), (override));
};