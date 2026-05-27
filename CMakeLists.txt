idf_component_register(
        SRCS
                "src/core/serial_comm_dispatcher.cpp"
                "src/core/serial_comm_protocol.cpp"
                "src/core/serial_comm_watchdog.cpp"
                "src/core/serial_comm_parser.cpp"
                "src/core/serial_comm.cpp"

                "src/middleware/serial_comm_transaction_manager.cpp"
                "src/middleware/serial_comm_manager_service.cpp"
                "src/middleware/serial_comm_manager_action.cpp"
                "src/middleware/serial_comm_manager_topic.cpp"
                "src/middleware/serial_comm_manager.cpp"

                "src/transport/uart_serial_comm.cpp"

        INCLUDE_DIRS
                "include"
                "include/serial_comm"
                "include/serial_comm/core"
                "include/serial_comm/messages"
                "include/serial_comm/middleware"
                "include/serial_comm/middleware/service"
                "include/serial_comm/middleware/action"
                "include/serial_comm/middleware/topic"
                "include/serial_comm/transport"

        REQUIRES
                freertos
                esp_timer
                driver
                log
)

target_compile_features(${COMPONENT_LIB} PUBLIC cxx_std_17)