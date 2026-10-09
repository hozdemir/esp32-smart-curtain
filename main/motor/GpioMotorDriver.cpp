#include "motor/GpioMotorDriver.hpp"

#include <cstdint>

namespace curtain {

GpioMotorDriver::GpioMotorDriver(gpio_num_t inputOne, gpio_num_t inputTwo, gpio_num_t sleepPin)
    : inputOne_{inputOne}, inputTwo_{inputTwo}, sleepPin_{sleepPin} {}

Result GpioMotorDriver::initialize() {
    if (inputOne_ == inputTwo_ || inputOne_ == sleepPin_ || inputTwo_ == sleepPin_ ||
        !GPIO_IS_VALID_OUTPUT_GPIO(inputOne_) || !GPIO_IS_VALID_OUTPUT_GPIO(inputTwo_) ||
        !GPIO_IS_VALID_OUTPUT_GPIO(sleepPin_)) {
        return Result::failure(ErrorCode::InvalidArgument);
    }

    // Configure nSLEEP first so the bridge cannot drive the motor while its
    // direction inputs are being initialized.
    const gpio_config_t sleepConfig{
        .pin_bit_mask = (1ULL << static_cast<unsigned>(sleepPin_)),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_ENABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    if (gpio_config(&sleepConfig) != ESP_OK || gpio_set_level(sleepPin_, 0U) != ESP_OK) {
        return Result::failure(ErrorCode::HardwareFailure);
    }

    const gpio_config_t inputConfig{
        .pin_bit_mask = (1ULL << static_cast<unsigned>(inputOne_)) |
                        (1ULL << static_cast<unsigned>(inputTwo_)),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_ENABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    if (gpio_config(&inputConfig) != ESP_OK || gpio_set_level(inputOne_, 0U) != ESP_OK ||
        gpio_set_level(inputTwo_, 0U) != ESP_OK) {
        disableOutputs();
        return Result::failure(ErrorCode::HardwareFailure);
    }

    initialized_ = true;
    running_ = false;
    return Result::success();
}

Result GpioMotorDriver::startOpening() {
    return setOutputs(1U, 0U);
}

Result GpioMotorDriver::startClosing() {
    return setOutputs(0U, 1U);
}

Result GpioMotorDriver::stop() {
    return setOutputs(0U, 0U);
}

bool GpioMotorDriver::isRunning() const {
    return running_;
}

Result GpioMotorDriver::setOutputs(uint32_t inputOneLevel, uint32_t inputTwoLevel) {
    if (!initialized_) {
        return Result::failure(ErrorCode::NotInitialized);
    }

    // Coast first so reversing direction never briefly shorts the H-bridge.
    if (gpio_set_level(inputOne_, 0U) != ESP_OK || gpio_set_level(inputTwo_, 0U) != ESP_OK) {
        disableOutputs();
        return Result::failure(ErrorCode::HardwareFailure);
    }

    if (inputOneLevel == 0U && inputTwoLevel == 0U) {
        if (gpio_set_level(sleepPin_, 0U) != ESP_OK) {
            disableOutputs();
            return Result::failure(ErrorCode::HardwareFailure);
        }
        running_ = false;
        return Result::success();
    }

    if (gpio_set_level(inputOne_, inputOneLevel) != ESP_OK ||
        gpio_set_level(inputTwo_, inputTwoLevel) != ESP_OK ||
        gpio_set_level(sleepPin_, 1U) != ESP_OK) {
        disableOutputs();
        return Result::failure(ErrorCode::HardwareFailure);
    }

    running_ = true;
    return Result::success();
}

void GpioMotorDriver::disableOutputs() {
    (void)gpio_set_level(inputOne_, 0U);
    (void)gpio_set_level(inputTwo_, 0U);
    (void)gpio_set_level(sleepPin_, 0U);
    running_ = false;
}

}  // namespace curtain
