idf_component_register(
        SRCS
                "serial_comm_eventloop.cpp"
                
                "core/serial_comm_dispatcher.cpp"
                "core/serial_comm_protocol.cpp"
                "core/serial_comm_watchdog.cpp"
                "core/serial_comm_parser.cpp"
                "core/serial_comm.cpp"

                "middleware/serial_comm_transaction_manager.cpp"
                "middleware/serial_comm_manager_service.cpp"
                "middleware/serial_comm_manager_action.cpp"
                "middleware/serial_comm_manager_topic.cpp"
                "middleware/serial_comm_manager.cpp"

                "middleware/service/serial_comm_service.cpp"
                "middleware/action/serial_comm_action.cpp"
                "middleware/topic/serial_comm_topic.cpp"

                "transport/uart_serial_comm.cpp"

        INCLUDE_DIRS
                "."
                "core"
                "messages"
                "middleware"
                "transport"

        REQUIRES
                freertos
                esp_timer
                driver
                log
)

target_compile_features(${COMPONENT_LIB} PUBLIC cxx_std_17)