/*
 * led_driver.hpp
 *
 *  Created on: Jul 15, 2024
 *      Author: ronik
 */

#ifndef INC_LED_DRIVER_HPP_
#define INC_LED_DRIVER_HPP_

#include "driver_interface.hpp"

class LEDDriver : public DriverInterface {
public:
    virtual void flashLED() override;
    virtual void wait(uint32_t timeout_ms) override;
};

#endif /* INC_LED_DRIVER_HPP_ */
