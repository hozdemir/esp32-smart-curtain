#pragma once

#include <array>
#include <atomic>
#include <cstdint>

#include "app/CurtainController.hpp"
#include "core/Result.hpp"
#include "esp_zigbee.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "freertos/timers.h"

namespace curtain {

/**
 * Exposes the curtain through standard Zigbee clusters while keeping stack callbacks
 * outside the business-logic thread.
 */
class ZigbeeController {
public:
    explicit ZigbeeController(CurtainController& curtainController);

    [[nodiscard]] Result start();
    void processPendingCommands();
    void publishCurtainState(CurtainState state, float openPercent);
    [[nodiscard]] Result publishBatteryState(float percentage, float voltage);
    [[nodiscard]] bool isJoined() const;

private:
    enum class CommandType : uint8_t {
        Open,
        Close,
        Stop,
        MoveTo,
    };

    struct Command {
        CommandType type;
        float targetPercent;
    };

    static constexpr uint8_t kEndpointId = 1U;
    static constexpr std::size_t kCommandQueueLength = 8U;
    static constexpr std::size_t kZigbeeTaskStackDepth = 6144U;

    static void zigbeeTaskEntry(void* context);
    static void zclActionHandler(uint32_t callbackId, void* message);
    static bool appSignalHandler(const ezb_app_signal_t* appSignal);
    static void retryTimerCallback(TimerHandle_t timer);
    static void retryCommissioningCallback(void* context);
    static void publishPositionCallback(void* context);
    static void publishBatteryCallback(void* context);

    void zigbeeTask();
    void handleMovementCommand(void* message);
    void handleAppSignal(const ezb_app_signal_t* appSignal);
    void scheduleCommissioningRetry(uint8_t mode);
    [[nodiscard]] bool enqueue(Command command);
    [[nodiscard]] Result initializeStorage();
    [[nodiscard]] Result createDeviceModel();
    void updatePositionAttribute();
    void updateBatteryAttributes();

    CurtainController& curtainController_;
    QueueHandle_t commandQueue_{nullptr};
    StaticQueue_t commandQueueControl_{};
    std::array<uint8_t, sizeof(Command) * kCommandQueueLength> commandQueueStorage_{};
    StaticTask_t zigbeeTaskControl_{};
    std::array<StackType_t, kZigbeeTaskStackDepth> zigbeeTaskStack_{};
    StaticTimer_t retryTimerControl_{};
    TimerHandle_t retryTimer_{nullptr};
    std::atomic<bool> started_{false};
    std::atomic<bool> joined_{false};
    std::atomic<bool> positionUpdateQueued_{false};
    std::atomic<bool> batteryUpdateQueued_{false};
    std::atomic<uint8_t> latestLiftPercentage_{100U};
    std::atomic<uint8_t> latestBatteryPercentageHalfUnits_{0xFFU};
    std::atomic<uint8_t> latestBatteryVoltageDecivolts_{0xFFU};
    std::atomic<uint8_t> retryMode_{0U};

    // The C stack callback API has no user context; one active Zigbee stack is supported.
    static ZigbeeController* activeInstance_;
};

}  // namespace curtain
