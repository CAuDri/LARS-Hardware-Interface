cmake_minimum_required(VERSION 3.22)

project(FirmwareHostTests LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

enable_testing()

function(add_logger_interface_test name output level error warning info debug)
    add_executable(${name} logger_interface_test.cpp)
    target_include_directories(${name} PRIVATE
        ../libraries/common/include
        ../devices/HardwareInterface/config
    )
    target_compile_definitions(${name} PRIVATE
        DEBUG_LOG_OUTPUT=${output}
        DEBUG_LOG_LEVEL=${level}
        TEST_ERROR_ENABLED=${error}
        TEST_WARNING_ENABLED=${warning}
        TEST_INFO_ENABLED=${info}
        TEST_DEBUG_ENABLED=${debug}
    )
    add_test(NAME ${name} COMMAND ${name})
endfunction()

add_logger_interface_test(logger_debug_usb LOG_OUTPUT_USB_CDC LOG_LEVEL_DEBUG 1 1 1 1)
add_logger_interface_test(logger_warning_uart LOG_OUTPUT_UART LOG_LEVEL_WARNING 1 1 0 0)
add_logger_interface_test(logger_output_none LOG_OUTPUT_NONE LOG_LEVEL_DEBUG 0 0 0 0)
