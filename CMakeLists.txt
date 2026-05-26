cmake_minimum_required(VERSION 3.16)

if(NOT DEFINED ENV{IDF_PATH} OR "$ENV{IDF_PATH}" STREQUAL "")
	set(ENV{IDF_PATH} "/home/iosnaaente/esp/v5.5.1/esp-idf")
endif()

if(NOT DEFINED ENV{IDF_PYTHON_ENV_PATH} OR "$ENV{IDF_PYTHON_ENV_PATH}" STREQUAL "")
	set(ENV{IDF_PYTHON_ENV_PATH} "/home/iosnaaente/.espressif/python_env/idf5.5_py3.14_env")
endif()

set(ENV{PATH} "$ENV{IDF_PYTHON_ENV_PATH}/bin:$ENV{PATH}")

include($ENV{IDF_PATH}/tools/cmake/project.cmake)

idf_build_set_property(
    MINIMAL_BUILD ON
)

project(SerialComm)
