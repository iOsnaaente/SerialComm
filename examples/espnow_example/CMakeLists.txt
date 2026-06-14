cmake_minimum_required(VERSION 3.16)

# EXTRA_COMPONENT_DIRS must point to the directory that CONTAINS the SerialComm
# component folder (i.e., the parent of SerialComm/).
# Resolve three levels up from  examples/espnow_example/  →  SerialComm parent.
get_filename_component(SERIAL_COMM_PARENT
    "${CMAKE_CURRENT_SOURCE_DIR}/../../.."
    ABSOLUTE
)
set(EXTRA_COMPONENT_DIRS "${SERIAL_COMM_PARENT}")

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(espnow_example)
