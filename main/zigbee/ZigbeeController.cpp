#include "zigbee/ZigbeeController.hpp"

#include <algorithm>
#include <cmath>

#include "esp_err.h"
#include "esp_log.h"
#include "esp_zigbee.h"
#include "ezbee/platform/radio.h"
#include "ezbee/zha.h"
#include "nvs_flash.h"

namespace curtain {
namespace {
constexpr char kLogTag[] = "ZigbeeController";
constexpr char kStoragePartition[] = "zb_storage";
constexpr char kManufacturerName[] = "\x0c" "SmartCurtain";
constexpr char kModelIdentifier[] = "\x10" "ESP32-C6-CURTAIN";
constexpr uint32_t kAllChannels = EZB_RADIO_2P4GHZ_ALL_CHANNEL_MASK;
constexpr TickType_t kRetryDelayTicks = pdMS_TO_TICKS(1000U);

esp_zigbee_config_t makeZigbeeConfig() {
    esp_zigbee_config_t config{};
    config.device_config.device_type = EZB_NWK_DEVICE_TYPE_END_DEVICE;
    config.device_config.install_code_policy = false;
    config.device_config.zed_config.ed_timeout = EZB_NWK_ED_TIMEOUT_64MIN;
    config.device_config.zed_config.keep_alive = 4000U;
    config.platform_config.storage_partition_name = kStoragePartition;
    config.platform_config.radio_config.radio_mode = ESP_ZIGBEE_RADIO_MODE_NATIVE;
    return config;
}

Result resultFromEspError(esp_err_t error) {
    return error == ESP_OK ? Result::success() : Result::failure(ErrorCode::HardwareFailure);
}
}  // namespace

ZigbeeController* ZigbeeController::activeInstance_ = nullptr;

ZigbeeController::ZigbeeController(CurtainController& curtainController)
    : curtainController_{curtainController} {
    commandQueue_ = xQueueCreateStatic(kCommandQueueLength, sizeof(Command), commandQueueStorage_.data(),
                                       &commandQueueControl_);
    retryTimer_ = xTimerCreateStatic("ZigbeeRetry", kRetryDelayTicks, pdFALSE, this,
                                     &ZigbeeController::retryTimerCallback, &retryTimerControl_);
}

Result ZigbeeController::start() {
    if (started_.load() || activeInstance_ != nullptr || commandQueue_ == nullptr || retryTimer_ == nullptr) {
        return Result::failure(ErrorCode::InvalidArgument);
    }

    const Result storageResult = initializeStorage();
    if (!storageResult) {
        ESP_LOGE(kLogTag, "NVS initialization failed");
        return storageResult;
    }

    activeInstance_ = this;
    TaskHandle_t task = xTaskCreateStatic(&ZigbeeController::zigbeeTaskEntry, "ZigbeeMain",
                                          zigbeeTaskStack_.size(), this, 5U, zigbeeTaskStack_.data(),
                                          &zigbeeTaskControl_);
    if (task == nullptr) {
        activeInstance_ = nullptr;
        return Result::failure(ErrorCode::HardwareFailure);
    }

    started_.store(true);
    return Result::success();
}

void ZigbeeController::processPendingCommands() {
    Command command{};
    while (xQueueReceive(commandQueue_, &command, 0U) == pdTRUE) {
        Result result = Result::failure(ErrorCode::InvalidArgument);
        switch (command.type) {
            case CommandType::Open:
                result = curtainController_.open();
                break;
            case CommandType::Close:
                result = curtainController_.close();
                break;
            case CommandType::Stop:
                result = curtainController_.stop();
                break;
            case CommandType::MoveTo:
                result = curtainController_.moveTo(command.targetPercent);
                break;
        }
        if (!result) {
            ESP_LOGE(kLogTag, "Curtain command failed (error %d)", static_cast<int>(result.error()));
        }
    }
}

void ZigbeeController::publishCurtainState(CurtainState, float openPercent) {
    if (!started_.load() || !std::isfinite(openPercent)) {
        return;
    }

    const auto clampedOpen = static_cast<uint8_t>(std::lround(std::clamp(openPercent, 0.0F, 100.0F)));
    // ZCL lift percentage is percentage closed: 0=open and 100=closed.
    latestLiftPercentage_.store(static_cast<uint8_t>(100U - clampedOpen));
    if (!positionUpdateQueued_.exchange(true)) {
        if (esp_zigbee_task_queue_post(&ZigbeeController::publishPositionCallback, this) != ESP_OK) {
            positionUpdateQueued_.store(false);
            ESP_LOGW(kLogTag, "Could not queue position attribute update");
        }
    }
}

Result ZigbeeController::publishBatteryState(float percentage, float voltage) {
    if (!started_.load() || !std::isfinite(percentage) || !std::isfinite(voltage) || voltage < 0.0F) {
        return Result::failure(ErrorCode::InvalidArgument);
    }

    // ZCL represents battery percentage in half-percent units and voltage in 100 mV units.
    latestBatteryPercentageHalfUnits_.store(
        static_cast<uint8_t>(std::lround(std::clamp(percentage, 0.0F, 100.0F) * 2.0F)));
    latestBatteryVoltageDecivolts_.store(
        static_cast<uint8_t>(std::lround(std::clamp(voltage * 10.0F, 0.0F, 254.0F))));

    if (!batteryUpdateQueued_.exchange(true) &&
        esp_zigbee_task_queue_post(&ZigbeeController::publishBatteryCallback, this) != ESP_OK) {
        batteryUpdateQueued_.store(false);
        return Result::failure(ErrorCode::HardwareFailure);
    }
    return Result::success();
}

bool ZigbeeController::isJoined() const {
    return joined_.load();
}

void ZigbeeController::zigbeeTaskEntry(void* context) {
    static_cast<ZigbeeController*>(context)->zigbeeTask();
}

void ZigbeeController::zclActionHandler(uint32_t callbackId, void* message) {
    if (activeInstance_ == nullptr) {
        return;
    }
    if (callbackId == EZB_ZCL_CORE_WINDOW_COVERING_MOVEMENT_CB_ID) {
        activeInstance_->handleMovementCommand(message);
    }
}

bool ZigbeeController::appSignalHandler(const ezb_app_signal_t* appSignal) {
    if (activeInstance_ != nullptr) {
        activeInstance_->handleAppSignal(appSignal);
    }
    return true;
}

void ZigbeeController::retryTimerCallback(TimerHandle_t timer) {
    auto* self = static_cast<ZigbeeController*>(pvTimerGetTimerID(timer));
    if (esp_zigbee_task_queue_post(&ZigbeeController::retryCommissioningCallback, self) != ESP_OK) {
        ESP_LOGE(kLogTag, "Could not queue commissioning retry");
    }
}

void ZigbeeController::retryCommissioningCallback(void* context) {
    auto* self = static_cast<ZigbeeController*>(context);
    const ezb_err_t error = ezb_bdb_start_top_level_commissioning(self->retryMode_.load());
    if (error != EZB_ERR_NONE) {
        ESP_LOGE(kLogTag, "Commissioning retry failed to start (0x%04x)", error);
    }
}

void ZigbeeController::publishPositionCallback(void* context) {
    auto* self = static_cast<ZigbeeController*>(context);
    self->positionUpdateQueued_.store(false);
    self->updatePositionAttribute();
}

void ZigbeeController::publishBatteryCallback(void* context) {
    auto* self = static_cast<ZigbeeController*>(context);
    self->batteryUpdateQueued_.store(false);
    self->updateBatteryAttributes();
}

void ZigbeeController::zigbeeTask() {
    const esp_zigbee_config_t config = makeZigbeeConfig();
    if (esp_zigbee_init(&config) != ESP_OK) {
        ESP_LOGE(kLogTag, "Zigbee stack initialization failed");
        vTaskDelete(nullptr);
        return;
    }

    ezb_aps_secur_enable_distributed_security(false);
    if (ezb_bdb_set_primary_channel_set(kAllChannels) != EZB_ERR_NONE ||
        ezb_bdb_set_secondary_channel_set(0U) != EZB_ERR_NONE ||
        ezb_app_signal_add_handler(&ZigbeeController::appSignalHandler) != EZB_ERR_NONE ||
        !createDeviceModel()) {
        ESP_LOGE(kLogTag, "Zigbee device setup failed");
        esp_zigbee_deinit();
        vTaskDelete(nullptr);
        return;
    }

    ESP_LOGI(kLogTag, "Starting Zigbee Window Covering endpoint %u", kEndpointId);
    if (esp_zigbee_start(false) != ESP_OK) {
        ESP_LOGE(kLogTag, "Zigbee stack start failed");
        esp_zigbee_deinit();
        vTaskDelete(nullptr);
        return;
    }

    (void)esp_zigbee_launch_mainloop();
    esp_zigbee_deinit();
    vTaskDelete(nullptr);
}

void ZigbeeController::handleMovementCommand(void* rawMessage) {
    auto* message = static_cast<ezb_zcl_window_covering_movement_message_t*>(rawMessage);
    if (message == nullptr || message->in.header == nullptr || message->info.dst_ep != kEndpointId ||
        message->info.cluster_id != EZB_ZCL_CLUSTER_ID_WINDOW_COVERING) {
        if (message != nullptr) {
            message->out.result = EZB_ZCL_STATUS_INVALID_FIELD;
        }
        return;
    }

    Command command{};
    switch (message->in.header->cmd_id) {
        case EZB_ZCL_CMD_WINDOW_COVERING_UP_OPEN_ID:
            command = {CommandType::Open, 100.0F};
            break;
        case EZB_ZCL_CMD_WINDOW_COVERING_DOWN_CLOSE_ID:
            command = {CommandType::Close, 0.0F};
            break;
        case EZB_ZCL_CMD_WINDOW_COVERING_STOP_ID:
            command = {CommandType::Stop, 0.0F};
            break;
        case EZB_ZCL_CMD_WINDOW_COVERING_GO_TO_LIFT_PERCENTAGE_ID:
            if (message->in.payload.lift_percentage > 100U) {
                message->out.result = EZB_ZCL_STATUS_INVALID_VALUE;
                return;
            }
            command = {CommandType::MoveTo,
                       static_cast<float>(100U - message->in.payload.lift_percentage)};
            break;
        default:
            message->out.result = EZB_ZCL_STATUS_UNSUP_CMD;
            return;
    }

    message->out.result = enqueue(command) ? EZB_ZCL_STATUS_SUCCESS : EZB_ZCL_STATUS_FAIL;
}

void ZigbeeController::handleAppSignal(const ezb_app_signal_t* appSignal) {
    const ezb_app_signal_type_t type = ezb_app_signal_get_type(appSignal);
    switch (type) {
        case EZB_ZDO_SIGNAL_SKIP_STARTUP:
            (void)ezb_bdb_start_top_level_commissioning(EZB_BDB_MODE_INITIALIZATION);
            break;
        case EZB_BDB_SIGNAL_DEVICE_FIRST_START:
        case EZB_BDB_SIGNAL_DEVICE_REBOOT: {
            const auto status = *static_cast<const ezb_bdb_comm_status_t*>(ezb_app_signal_get_params(appSignal));
            if (status == EZB_BDB_STATUS_SUCCESS) {
                if (ezb_bdb_is_factory_new()) {
                    (void)ezb_bdb_start_top_level_commissioning(EZB_BDB_MODE_NETWORK_STEERING);
                } else {
                    joined_.store(true);
                    ESP_LOGI(kLogTag, "Restored existing Zigbee network");
                }
            } else {
                scheduleCommissioningRetry(EZB_BDB_MODE_INITIALIZATION);
            }
            break;
        }
        case EZB_BDB_SIGNAL_STEERING: {
            const auto status = *static_cast<const ezb_bdb_comm_status_t*>(ezb_app_signal_get_params(appSignal));
            if (status == EZB_BDB_STATUS_SUCCESS) {
                joined_.store(true);
                ESP_LOGI(kLogTag, "Joined Zigbee network: PAN=0x%04x channel=%u address=0x%04x",
                         ezb_nwk_get_panid(), ezb_nwk_get_current_channel(), ezb_nwk_get_short_address());
                updatePositionAttribute();
                updateBatteryAttributes();
            } else {
                joined_.store(false);
                ESP_LOGW(kLogTag, "Zigbee network join failed (0x%02x), retrying", status);
                scheduleCommissioningRetry(EZB_BDB_MODE_NETWORK_STEERING);
            }
            break;
        }
        case EZB_ZDO_SIGNAL_LEAVE:
            joined_.store(false);
            ESP_LOGW(kLogTag, "Left Zigbee network");
            scheduleCommissioningRetry(EZB_BDB_MODE_NETWORK_STEERING);
            break;
        default:
            ESP_LOGD(kLogTag, "Zigbee signal: %s", ezb_app_signal_to_string(type));
            break;
    }
}

void ZigbeeController::scheduleCommissioningRetry(uint8_t mode) {
    retryMode_.store(mode);
    if (xTimerStart(retryTimer_, 0U) != pdPASS) {
        ESP_LOGE(kLogTag, "Could not start commissioning retry timer");
    }
}

bool ZigbeeController::enqueue(Command command) {
    return xQueueSend(commandQueue_, &command, 0U) == pdTRUE;
}

Result ZigbeeController::initializeStorage() {
    esp_err_t error = nvs_flash_init();
    if (error == ESP_ERR_NVS_NO_FREE_PAGES || error == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        error = nvs_flash_erase();
        if (error == ESP_OK) {
            error = nvs_flash_init();
        }
    }
    if (error != ESP_OK) {
        return resultFromEspError(error);
    }

    error = nvs_flash_init_partition(kStoragePartition);
    if (error == ESP_ERR_NVS_NO_FREE_PAGES || error == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        error = nvs_flash_erase_partition(kStoragePartition);
        if (error == ESP_OK) {
            error = nvs_flash_init_partition(kStoragePartition);
        }
    }
    return resultFromEspError(error);
}

Result ZigbeeController::createDeviceModel() {
    ezb_af_device_desc_t device = ezb_af_create_device_desc();
    ezb_zha_window_covering_config_t config{};
    config.basic_cfg.zcl_version = EZB_ZCL_BASIC_ZCL_VERSION_DEFAULT_VALUE;
    config.basic_cfg.power_source = EZB_ZCL_BASIC_POWER_SOURCE_BATTERY;
    config.identify_cfg.identify_time = EZB_ZCL_IDENTIFY_IDENTIFY_TIME_DEFAULT_VALUE;
    config.window_covering_cfg.window_covering_type =
        EZB_ZCL_WINDOW_COVERING_WINDOW_COVERING_TYPE_DRAPERY;
    config.window_covering_cfg.config_status =
        EZB_ZCL_WINDOW_COVERING_CONFIG_STATUS_OPERATIONAL |
        EZB_ZCL_WINDOW_COVERING_CONFIG_STATUS_ONLINE;
    config.window_covering_cfg.mode = 0U;
    config.groups_cfg.name_support = EZB_ZCL_GROUPS_NAME_SUPPORT_DEFAULT_VALUE;
    config.scenes_cfg.scene_count = EZB_ZCL_SCENES_SCENE_COUNT_DEFAULT_VALUE;
    config.scenes_cfg.current_scene = EZB_ZCL_SCENES_CURRENT_SCENE_DEFAULT_VALUE;
    config.scenes_cfg.current_group = EZB_ZCL_SCENES_CURRENT_GROUP_DEFAULT_VALUE;
    config.scenes_cfg.scene_valid = EZB_ZCL_SCENES_SCENE_VALID_DEFAULT_VALUE;
    config.scenes_cfg.name_support = EZB_ZCL_SCENES_NAME_SUPPORT_DEFAULT_VALUE;
    config.scenes_cfg.scene_table_size = EZB_ZCL_SCENES_SCENE_TABLE_SIZE_DEFAULT_VALUE;

    ezb_af_ep_desc_t endpoint = ezb_zha_create_window_covering(kEndpointId, &config);
    if (device == EZB_INVALID_AF_DEVICE_DESC || endpoint == EZB_INVALID_AF_EP_DESC) {
        return Result::failure(ErrorCode::HardwareFailure);
    }

    ezb_zcl_cluster_desc_t basic =
        ezb_af_endpoint_get_cluster_desc(endpoint, EZB_ZCL_CLUSTER_ID_BASIC, EZB_ZCL_CLUSTER_SERVER);
    ezb_zcl_cluster_desc_t covering =
        ezb_af_endpoint_get_cluster_desc(endpoint, EZB_ZCL_CLUSTER_ID_WINDOW_COVERING, EZB_ZCL_CLUSTER_SERVER);
    uint8_t liftPercentage = latestLiftPercentage_.load();
    if (basic == EZB_INVALID_ZCL_CLUSTER_DESC || covering == EZB_INVALID_ZCL_CLUSTER_DESC ||
        ezb_zcl_basic_cluster_desc_add_attr(basic, EZB_ZCL_ATTR_BASIC_MANUFACTURER_NAME_ID,
                                            kManufacturerName) != EZB_ERR_NONE ||
        ezb_zcl_basic_cluster_desc_add_attr(basic, EZB_ZCL_ATTR_BASIC_MODEL_IDENTIFIER_ID,
                                            kModelIdentifier) != EZB_ERR_NONE ||
        ezb_zcl_window_covering_cluster_desc_add_attr(
            covering, EZB_ZCL_ATTR_WINDOW_COVERING_CURRENT_POSITION_LIFT_PERCENTAGE_ID,
            &liftPercentage) != EZB_ERR_NONE) {
        return Result::failure(ErrorCode::HardwareFailure);
    }

    const ezb_zcl_power_config_cluster_server_config_t powerConfig{
        .mains_voltage = 0U,
        .mains_voltage_min_threshold = 0U,
        .mains_voltage_max_threshold = 0xFFFFU,
    };
    ezb_zcl_cluster_desc_t power =
        ezb_zcl_power_config_create_cluster_desc(&powerConfig, EZB_ZCL_CLUSTER_SERVER);
    uint8_t batteryPercentage = latestBatteryPercentageHalfUnits_.load();
    uint8_t batteryVoltage = latestBatteryVoltageDecivolts_.load();
    uint8_t batterySize = EZB_ZCL_POWER_CONFIG_BATTERY_SIZE_BUILD_IN;
    uint8_t batteryQuantity = 1U;
    if (power == EZB_INVALID_ZCL_CLUSTER_DESC ||
        ezb_zcl_power_config_cluster_desc_add_attr(
            power, EZB_ZCL_ATTR_POWER_CONFIG_BATTERY_PERCENTAGE_REMAINING_ID,
            &batteryPercentage) != EZB_ERR_NONE ||
        ezb_zcl_power_config_cluster_desc_add_attr(
            power, EZB_ZCL_ATTR_POWER_CONFIG_BATTERY_VOLTAGE_ID, &batteryVoltage) != EZB_ERR_NONE ||
        ezb_zcl_power_config_cluster_desc_add_attr(
            power, EZB_ZCL_ATTR_POWER_CONFIG_BATTERY_SIZE_ID, &batterySize) != EZB_ERR_NONE ||
        ezb_zcl_power_config_cluster_desc_add_attr(
            power, EZB_ZCL_ATTR_POWER_CONFIG_BATTERY_QUANTITY_ID, &batteryQuantity) != EZB_ERR_NONE ||
        ezb_af_endpoint_add_cluster_desc(endpoint, power) != EZB_ERR_NONE ||
        ezb_af_device_add_endpoint_desc(device, endpoint) != EZB_ERR_NONE ||
        ezb_af_device_desc_register(device) != EZB_ERR_NONE) {
        return Result::failure(ErrorCode::HardwareFailure);
    }

    const ezb_af_node_power_desc_t nodePower{
        .u16 = static_cast<uint16_t>(
            EZB_AF_NODE_POWER_MODE_COME_ON_PERIODICALLY |
            (EZB_AF_NODE_POWER_SOURCE_RECHARGEABLE_BATTERY << 4U) |
            (EZB_AF_NODE_POWER_SOURCE_RECHARGEABLE_BATTERY << 8U) |
            (EZB_AF_NODE_POWER_SOURCE_LEVEL_100_PERCENT << 12U)),
    };
    if (ezb_af_set_node_power_desc(&nodePower) != EZB_ERR_NONE) {
        return Result::failure(ErrorCode::HardwareFailure);
    }

    ezb_zcl_core_action_handler_register(&ZigbeeController::zclActionHandler);
    return Result::success();
}

void ZigbeeController::updatePositionAttribute() {
    uint8_t value = latestLiftPercentage_.load();
    const ezb_zcl_status_t status = ezb_zcl_set_attr_value(
        kEndpointId, EZB_ZCL_CLUSTER_ID_WINDOW_COVERING, EZB_ZCL_CLUSTER_SERVER,
        EZB_ZCL_ATTR_WINDOW_COVERING_CURRENT_POSITION_LIFT_PERCENTAGE_ID,
        EZB_ZCL_STD_MANUF_CODE, &value, false);
    if (status != EZB_ZCL_STATUS_SUCCESS) {
        ESP_LOGW(kLogTag, "Position attribute update failed (0x%02x)", status);
    }
}

void ZigbeeController::updateBatteryAttributes() {
    uint8_t percentage = latestBatteryPercentageHalfUnits_.load();
    uint8_t voltage = latestBatteryVoltageDecivolts_.load();
    const ezb_zcl_status_t percentageStatus = ezb_zcl_set_attr_value(
        kEndpointId, EZB_ZCL_CLUSTER_ID_POWER_CONFIG, EZB_ZCL_CLUSTER_SERVER,
        EZB_ZCL_ATTR_POWER_CONFIG_BATTERY_PERCENTAGE_REMAINING_ID,
        EZB_ZCL_STD_MANUF_CODE, &percentage, false);
    const ezb_zcl_status_t voltageStatus = ezb_zcl_set_attr_value(
        kEndpointId, EZB_ZCL_CLUSTER_ID_POWER_CONFIG, EZB_ZCL_CLUSTER_SERVER,
        EZB_ZCL_ATTR_POWER_CONFIG_BATTERY_VOLTAGE_ID,
        EZB_ZCL_STD_MANUF_CODE, &voltage, false);
    if (percentageStatus != EZB_ZCL_STATUS_SUCCESS || voltageStatus != EZB_ZCL_STATUS_SUCCESS) {
        ESP_LOGW(kLogTag, "Battery attribute update failed (percentage=0x%02x voltage=0x%02x)",
                 percentageStatus, voltageStatus);
    }
}

}  // namespace curtain
