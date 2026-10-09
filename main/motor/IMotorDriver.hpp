#pragma once

#include "core/Result.hpp"

namespace curtain {

/** Separates curtain motion commands from the electrical motor implementation. */
class IMotorDriver {
public:
    virtual ~IMotorDriver() = default;

    [[nodiscard]] virtual Result startOpening() = 0;
    [[nodiscard]] virtual Result startClosing() = 0;
    [[nodiscard]] virtual Result stop() = 0;
    [[nodiscard]] virtual bool isRunning() const = 0;
};

}  // namespace curtain
