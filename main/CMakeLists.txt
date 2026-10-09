idf_component_register(
    SRCS
        "esp32-smart-curtain.cpp"
        "app/CurtainController.cpp"
        "motor/GpioMotorDriver.cpp"
        "position/SimulatedPositionSensor.cpp"
        "zigbee/ZigbeeController.cpp"
    INCLUDE_DIRS "."
    REQUIRES esp_driver_gpio nvs_flash espressif__esp-zigbee-lib
)
