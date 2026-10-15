# For more information about build system see
# https://docs.espressif.com/projects/esp-idf/en/latest/api-guides/build-system.html
# The following five lines of boilerplate have to be in your project's
# CMakeLists in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.16)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
# "Trim" the build. Include the minimal set of components, main, and anything it depends on.
idf_build_set_property(MINIMAL_BUILD ON)
project(grbl-joy)

# Strip known GCC-only flags from compile_commands.json so clangd does not report
# drv_unknown_argument errors (e.g. -fno-tree-switch-conversion) on ESP-IDF headers.
if(EXISTS "${CMAKE_SOURCE_DIR}/fix_compile_commands.py")
    add_custom_target(
        fix_clangd ALL
        COMMAND ${CMAKE_COMMAND} -E echo "Running fix_compile_commands.py..."
        COMMAND ${CMAKE_COMMAND} -E env python ${CMAKE_SOURCE_DIR}/fix_compile_commands.py
        COMMENT "Cleaning compile_commands.json for clangd"
        VERBATIM
    )
endif()
