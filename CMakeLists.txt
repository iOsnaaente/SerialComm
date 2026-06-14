cmake_minimum_required(VERSION 3.16)

find_package( 
        Python3 
        REQUIRED 
        COMPONENTS 
                Interpreter 
)

# USER APP IDL FILES
file(
        GLOB_RECURSE
        SERIAL_COMM_IDL_FILES

        "${CMAKE_CURRENT_SOURCE_DIR}/user_app/*.struct"
        "${CMAKE_CURRENT_SOURCE_DIR}/user_app/*.event"
        "${CMAKE_CURRENT_SOURCE_DIR}/user_app/*.request"
        "${CMAKE_CURRENT_SOURCE_DIR}/user_app/*.mission"
)

# GENERATED OUTPUT DIRECTORY
set(
        SERIAL_COMM_GENERATED_DIR
        "${CMAKE_CURRENT_SOURCE_DIR}/include/serial_comm/generated"
)

# BUILD DIRECTORY FOR CODEGEN
set(
        SERIAL_COMM_BUILD_DIR
        "${CMAKE_CURRENT_BINARY_DIR}/serial_comm_codegen"
)

# GENERATED STAMP FILE
set(
        SERIAL_COMM_GENERATED_STAMP
        "${SERIAL_COMM_GENERATED_DIR}/generated.stamp"
)

# COMPONENT SOURCES
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
                "src/transport/espnow_serial_comm.cpp"

        INCLUDE_DIRS
                "include"


        REQUIRES
                freertos
                esp_timer
                driver
                log
                esp_wifi
)


# CODE GENERATION
add_custom_command( 
        OUTPUT 
                ${SERIAL_COMM_GENERATED_STAMP} 
        
        COMMAND 
                ${CMAKE_COMMAND} -E make_directory 
                ${SERIAL_COMM_GENERATED_DIR} 
                
        COMMAND 
                ${CMAKE_COMMAND} -E make_directory 
                ${SERIAL_COMM_BUILD_DIR} 
                
        COMMAND 
                ${Python3_EXECUTABLE} 
                ${CMAKE_CURRENT_SOURCE_DIR}/user_app/generate.py 
                --input 
                        ${CMAKE_CURRENT_SOURCE_DIR}/user_app 
                --output 
                        ${SERIAL_COMM_GENERATED_DIR} 
                        
        COMMAND 
                ${CMAKE_COMMAND} -E touch 
                ${SERIAL_COMM_GENERATED_STAMP} 
                
        DEPENDS 
                ${SERIAL_COMM_IDL_FILES}
                ${CMAKE_CURRENT_SOURCE_DIR}/user_app/generate.py 
                
        COMMENT 
                "Generating SerialComm IDL structures" 
                
        VERBATIM 
)

# CODEGEN TARGET
add_custom_target(
        serial_comm_codegen
        DEPENDS
                ${SERIAL_COMM_GENERATED_STAMP}
)

# BUILD DEPENDENCY
add_dependencies(
        ${COMPONENT_LIB}
        serial_comm_codegen
)

# C++ STANDARD
target_compile_features(
        ${COMPONENT_LIB} 
        PUBLIC 
        cxx_std_17
)