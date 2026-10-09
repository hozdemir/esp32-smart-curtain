#include <cstdint>

#include "app/CurtainController.hpp"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "motor/GpioMotorDriver.hpp"
#include "position/SimulatedPositionSensor.hpp"
#include "zigbee/ZigbeeController.hpp"

namespace {
constexpr char kLogTag[] = "SmartCurtain";
constexpr gpio_num_t kMotorInputOneGpio = GPIO_NUM_1;  // XIAO D1 -> DRV8833 AIN1
constexpr gpio_num_t kMotorInputTwoGpio = GPIO_NUM_2;  // XIAO D2 -> DRV8833 AIN2
constexpr gpio_num_t kMotorSleepGpio = GPIO_NUM_21;    // XIAO D3 -> DRV8833 nSLEEP/STBY
constexpr float kSimulatedSpeedPercentPerSecond = 20.0F;
constexpr float kPositionTolerancePercent = 0.5F;
constexpr uint32_t kUpdatePeriodMs = 50U;
constexpr uint32_t kPublishPeriodMs = 250U;

const char* stateName(curtain::CurtainState state) {
    switch (state) {
        case curtain::CurtainState::Unknown: return "Unknown";
        case curtain::CurtainState::Open: return "Open";
        case curtain::CurtainState::Closed: return "Closed";
        case curtain::CurtainState::Opening: return "Opening";
        case curtain::CurtainState::Closing: return "Closing";
        case curtain::CurtainState::Stopped: return "Stopped";
        case curtain::CurtainState::Stalled: return "Stalled";
    }
    return "Invalid";
}

void runController(curtain::CurtainController& controller, curtain::ZigbeeController& zigbee) {
    const TickType_t updateTicks = pdMS_TO_TICKS(kUpdatePeriodMs);
    TickType_t previousTick = xTaskGetTickCount();
    TickType_t lastLogTick = previousTick;
    TickType_t lastPublishTick = previousTick;
    curtain::CurtainState lastState = controller.state();

    while (true) {
        vTaskDelay(updateTicks);
        const TickType_t now = xTaskGetTickCount();
        const uint32_t elapsedMs = static_cast<uint32_t>((now - previousTick) * portTICK_PERIOD_MS);
        previousTick = now;
        zigbee.processPendingCommands();
        controller.update(elapsedMs);

        const curtain::CurtainState currentState = controller.state();
        if (currentState != lastState || (now - lastPublishTick) >= pdMS_TO_TICKS(kPublishPeriodMs)) {
            zigbee.publishCurtainState(currentState, controller.positionPercent());
            lastState = currentState;
            lastPublishTick = now;
        }

        if ((now - lastLogTick) >= pdMS_TO_TICKS(1000U)) {
            ESP_LOGI(kLogTag, "zigbee=%s state=%s position=%.1f%%",
                     zigbee.isJoined() ? "joined" : "not-joined", stateName(currentState),
                     static_cast<double>(controller.positionPercent()));
            lastLogTick = now;
        }
    }
}

bool runCommand(const char* description, curtain::Result result) {
    if (!result) {
        ESP_LOGE(kLogTag, "%s failed (error %d)", description, static_cast<int>(result.error()));
        return false;
    }
    ESP_LOGI(kLogTag, "%s", description);
    return true;
}
}  // namespace

extern "C" void app_main() {
    curtain::GpioMotorDriver motor{kMotorInputOneGpio, kMotorInputTwoGpio, kMotorSleepGpio};
    curtain::SimulatedPositionSensor position{kSimulatedSpeedPercentPerSecond};

    if (!runCommand("Motor initialized in stopped state", motor.initialize())) {
        return;
    }

    curtain::CurtainController controller{motor, position, kPositionTolerancePercent};
    // ZigbeeController owns its static FreeRTOS task stack. Static storage keeps this
    // large buffer out of ESP-IDF's comparatively small app_main task stack.
    static curtain::ZigbeeController zigbee{controller};
    if (!runCommand("Zigbee controller started", zigbee.start())) {
        return;
    }
    zigbee.publishCurtainState(controller.state(), controller.positionPercent());
    runController(controller, zigbee);
}
