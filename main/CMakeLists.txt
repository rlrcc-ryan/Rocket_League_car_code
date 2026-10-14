set(srcs
        "main.c"
        "Bluetooth_handler.cpp"
        "servos_and_motors_handler.cpp")

set(requires "bluepad32" "bluepad32_arduino" "arduino" "btstack")

idf_component_register(
    SRCS ${srcs}
    INCLUDE_DIRS "."
    REQUIRES ${requires}
)
                    