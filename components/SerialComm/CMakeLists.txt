idf_component_register(
	SRCS
		"src/serial_comm.cpp"
		"src/serial_comm_crc.cpp"
		"src/serial_comm_dispatcher.cpp"
		"src/serial_comm_eventloop.cpp"
		"src/serial_comm_parser.cpp"
		"src/serial_comm_protocol.cpp"
		"src/serial_comm_ringbuff.cpp"
		"src/uart_serial_comm.cpp"
	

    INCLUDE_DIRS
		"include"


	REQUIRES 
        freertos 
        esp_timer 
        driver 
        log

        
)

target_compile_features(${COMPONENT_LIB} PUBLIC cxx_std_17) 