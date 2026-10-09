#pragma once

#include "driver/gpio.h"
#include "motor/IMotorDriver.hpp"

namespace curtain {

/** Drives a DRV8833-style H-bridge and sleeps it whenever the curtain is stopped. */
class GpioMotorDriver final : public IMotorDriver {
public:
    GpioMotorDriver(gpio_num_t inputOne, gpio_num_t inputTwo, gpio_num_t sleepPin);

    [[nodiscard]] Result initialize();
    [[nodiscard]] Result startOpening() override;
    [[nodiscard]] Result startClosing() override;
    [[nodiscard]] Result stop() override;
    [[nodiscard]] bool isRunning() const override;

private:
    [[nodiscard]] Result setOutputs(uint32_t inputOneLevel, uint32_t inputTwoLevel);
    void disableOutputs();

    gpio_num_t inputOne_;
    gpio_num_t inputTwo_;
    gpio_num_t sleepPin_;
    bool initialized_{false};
    bool running_{false};
};

}  // namespace curtain
