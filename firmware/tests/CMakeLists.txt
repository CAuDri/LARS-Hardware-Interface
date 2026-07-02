cmake_minimum_required(VERSION 3.22)

project(FirmwareHostTests LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

enable_testing()

add_executable(crsf_protocol_test crsf_protocol_test.cpp)
target_include_directories(crsf_protocol_test PRIVATE ../libraries/drivers/include)

add_test(NAME crsf_protocol_test COMMAND crsf_protocol_test)
