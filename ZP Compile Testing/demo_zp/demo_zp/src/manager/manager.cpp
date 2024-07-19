#include <cstdint>
#include "driver_interface.hpp"
#include "manager.hpp"

void Manager::flashOnce()
{
    LEDDriver->flashLED();
}

void Manager::flashTwice(uint32_t timeout_ms)
{
    LEDDriver->flashLED();
    LEDDriver->wait(timeout_ms);
    LEDDriver->flashLED();
}

Manager::Manager(DriverInterface *driverPtr) : LEDDriver(driverPtr)
{
    count = 0;
}

void Manager::runManager()
{
    if(count % 2 == 0)
    {
        flashOnce();
    }
    else
    {
        flashTwice(250);
    }

    ++count;
}
