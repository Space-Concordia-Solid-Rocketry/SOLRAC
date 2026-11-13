cmake_minimum_required(VERSION 3.20)
project(solrac VERSION 0.1.0 LANGUAGES C CXX)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

file(GLOB SOLRAC_SOURCES "src/*.cpp" "src/*.c")
file(GLOB DRIVER_SOURCES "drivers/src/*.cpp" "drivers/src/*.c")

add_library(solrac STATIC
    ${SOLRAC_SOURCES}
    ${DRIVER_SOURCES}
    dependancies/libcanard/canard.c
    dependancies/o1heap/o1heap.c
)

target_include_directories(solrac PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/inc
    ${CMAKE_CURRENT_SOURCE_DIR}/drivers/inc
    ${CMAKE_CURRENT_SOURCE_DIR}/dependancies/libcanard
    ${CMAKE_CURRENT_SOURCE_DIR}/dependancies/o1heap
)

# Enforce JSF AV C++ constraints on SOLRAC library source files
target_compile_options(solrac PRIVATE
    $<$<COMPILE_LANGUAGE:CXX>:
        -fno-exceptions
        -fno-rtti
        -Wall
        -Wextra
        -Wpedantic
        -Wshadow
        -Wconversion
        -Wold-style-cast
    >
)

# Native macOS/PC simulation tests when SOLRAC is opened standalone
if(CMAKE_PROJECT_NAME STREQUAL PROJECT_NAME)
    file(GLOB TEST_SOURCES "test/*.cpp")
    if(TEST_SOURCES)
        enable_testing()
        add_executable(solrac_tests ${TEST_SOURCES})
        target_link_libraries(solrac_tests PRIVATE solrac)
        target_compile_options(solrac_tests PRIVATE -fno-exceptions -fno-rtti)
        add_test(NAME SolracUnitTests COMMAND solrac_tests)
    endif()
endif()