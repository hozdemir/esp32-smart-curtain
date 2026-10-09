#include "position/SimulatedPositionSensor.hpp"

#include <algorithm>
#include <cmath>

namespace curtain {
namespace {
constexpr float kMillisecondsPerSecond = 1000.0F;
}

SimulatedPositionSensor::SimulatedPositionSensor(float movementPercentPerSecond,
                                                 float initialPositionPercent)
    : movementPercentPerSecond_{std::max(0.0F, movementPercentPerSecond)},
      positionPercent_{0.0F} {
    setPositionPercent(initialPositionPercent);
}

float SimulatedPositionSensor::positionPercent() const {
    return positionPercent_;
}

void SimulatedPositionSensor::updateOpening(uint32_t elapsedMs) {
    const float distance = movementPercentPerSecond_ * static_cast<float>(elapsedMs) /
                           kMillisecondsPerSecond;
    positionPercent_ = std::min(100.0F, positionPercent_ + distance);
}

void SimulatedPositionSensor::updateClosing(uint32_t elapsedMs) {
    const float distance = movementPercentPerSecond_ * static_cast<float>(elapsedMs) /
                           kMillisecondsPerSecond;
    positionPercent_ = std::max(0.0F, positionPercent_ - distance);
}

void SimulatedPositionSensor::setPositionPercent(float position) {
    if (std::isfinite(position)) {
        positionPercent_ = std::clamp(position, 0.0F, 100.0F);
    }
}

}  // namespace curtain
