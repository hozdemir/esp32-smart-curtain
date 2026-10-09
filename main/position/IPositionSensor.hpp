#pragma once

#include <cstdint>

namespace curtain {

/** Presents normalized curtain position so control logic is independent of sensing hardware. */
class IPositionSensor {
public:
    virtual ~IPositionSensor() = default;

    [[nodiscard]] virtual float positionPercent() const = 0;
    virtual void updateOpening(uint32_t elapsedMs) = 0;
    virtual void updateClosing(uint32_t elapsedMs) = 0;
    virtual void setPositionPercent(float position) = 0;
};

}  // namespace curtain
