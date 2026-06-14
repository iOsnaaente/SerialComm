cmake_minimum_required(VERSION 3.16)

# SerialComm is available via components/SerialComm symlink in this directory.
# No EXTRA_COMPONENT_DIRS needed — IDF finds components/ automatically.

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(uart_to_espnow_bridge)
