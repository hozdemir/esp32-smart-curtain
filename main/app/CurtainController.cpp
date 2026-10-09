#include "app/CurtainController.hpp"

#include <algorithm>
#include <cmath>

#include "esp_log.h"

namespace curtain {
namespace {
constexpr char kLogTag[] = "CurtainController";
}

CurtainController::CurtainController(IMotorDriver& motor, IPositionSensor& positionSensor,
                                     float positionTolerancePercent)
    : motor_{motor},
      positionSensor_{positionSensor},
      tolerancePercent_{std::clamp(positionTolerancePercent, 0.0F, 10.0F)} {
    const float position = positionSensor_.positionPercent();
    state_ = position <= tolerancePercent_ ? CurtainState::Closed
             : position >= 100.0F - tolerancePercent_ ? CurtainState::Open
                                                       : CurtainState::Stopped;
}

Result CurtainController::open() {
    return moveTo(100.0F);
}

Result CurtainController::close() {
    return moveTo(0.0F);
}

Result CurtainController::stop() {
    const Result result = motor_.stop();
    if (result) {
        state_ = CurtainState::Stopped;
    } else {
        ESP_LOGE(kLogTag, "Failed to stop motor (error %d)", static_cast<int>(result.error()));
    }
    return result;
}

Result CurtainController::moveTo(float targetPercent) {
    if (!std::isfinite(targetPercent)) {
        ESP_LOGE(kLogTag, "Rejected non-finite target position");
        return Result::failure(ErrorCode::InvalidArgument);
    }

    targetPercent_ = std::clamp(targetPercent, 0.0F, 100.0F);
    const float currentPosition = positionSensor_.positionPercent();
    if (targetReached(currentPosition)) {
        completeMovement();
        return motor_.isRunning() ? Result::failure(ErrorCode::HardwareFailure)
                                  : Result::success();
    }

    const bool opening = targetPercent_ > currentPosition;
    const Result result = opening ? motor_.startOpening() : motor_.startClosing();
    if (!result) {
        state_ = CurtainState::Stopped;
        ESP_LOGE(kLogTag, "Failed to start motor (error %d)", static_cast<int>(result.error()));
        return result;
    }
    state_ = opening ? CurtainState::Opening : CurtainState::Closing;
    return Result::success();
}

void CurtainController::update(uint32_t elapsedMs) {
    if (state_ == CurtainState::Opening) {
        positionSensor_.updateOpening(elapsedMs);
    } else if (state_ == CurtainState::Closing) {
        positionSensor_.updateClosing(elapsedMs);
    } else {
        return;
    }

    if (targetReached(positionSensor_.positionPercent())) {
        positionSensor_.setPositionPercent(targetPercent_);
        completeMovement();
    }
}

CurtainState CurtainController::state() const {
    return state_;
}

float CurtainController::positionPercent() const {
    return positionSensor_.positionPercent();
}

bool CurtainController::targetReached(float position) const {
    return std::fabs(position - targetPercent_) <= tolerancePercent_ ||
           (state_ == CurtainState::Opening && position >= targetPercent_) ||
           (state_ == CurtainState::Closing && position <= targetPercent_);
}

void CurtainController::completeMovement() {
    const Result result = motor_.stop();
    if (!result) {
        state_ = CurtainState::Stopped;
        ESP_LOGE(kLogTag, "Failed to stop at target (error %d)", static_cast<int>(result.error()));
        return;
    }

    state_ = targetPercent_ <= tolerancePercent_ ? CurtainState::Closed
             : targetPercent_ >= 100.0F - tolerancePercent_ ? CurtainState::Open
                                                            : CurtainState::Stopped;
}

}  // namespace curtain
