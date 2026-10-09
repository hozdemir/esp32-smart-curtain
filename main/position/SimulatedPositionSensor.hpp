#pragma once

#include "position/IPositionSensor.hpp"

namespace curtain {

/** Enables hardware motor testing before a physical position sensor is selected. */
class SimulatedPositionSensor final : public IPositionSensor {
public:
    explicit SimulatedPositionSensor(float movementPercentPerSecond,
                                     float initialPositionPercent = 0.0F);

    [[nodiscard]] float positionPercent() const override;
    void updateOpening(uint32_t elapsedMs) override;
    void updateClosing(uint32_t elapsedMs) override;
    void setPositionPercent(float position) override;

private:
    float movementPercentPerSecond_;
    float positionPercent_;
};

}  // namespace curtain
