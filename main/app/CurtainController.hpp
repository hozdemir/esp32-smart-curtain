#pragma once

#include <cstdint>

#include "core/CurtainState.hpp"
#include "core/Result.hpp"
#include "motor/IMotorDriver.hpp"
#include "position/IPositionSensor.hpp"

namespace curtain {

/** Coordinates target-based movement without depending on a particular motor or sensor. */
class CurtainController {
public:
    CurtainController(IMotorDriver& motor, IPositionSensor& positionSensor,
                      float positionTolerancePercent = 0.5F);

    [[nodiscard]] Result open();
    [[nodiscard]] Result close();
    [[nodiscard]] Result stop();
    [[nodiscard]] Result moveTo(float targetPercent);
    void update(uint32_t elapsedMs);
    [[nodiscard]] CurtainState state() const;
    [[nodiscard]] float positionPercent() const;

private:
    [[nodiscard]] bool targetReached(float position) const;
    void completeMovement();

    IMotorDriver& motor_;
    IPositionSensor& positionSensor_;
    float tolerancePercent_;
    float targetPercent_{0.0F};
    CurtainState state_{CurtainState::Unknown};
};

}  // namespace curtain
