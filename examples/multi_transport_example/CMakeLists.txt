cmake_minimum_required(VERSION 3.16)

get_filename_component(SERIAL_COMM_PARENT
    "${CMAKE_CURRENT_SOURCE_DIR}/../../.."
    ABSOLUTE
)
set(EXTRA_COMPONENT_DIRS "${SERIAL_COMM_PARENT}")

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(multi_transport_example)
