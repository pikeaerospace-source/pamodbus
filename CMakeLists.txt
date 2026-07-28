cmake_minimum_required(VERSION 3.10)
project(pamodbus VERSION 1.0.0 LANGUAGES C)

# C99 standard
set(CMAKE_C_STANDARD 99)
set(CMAKE_C_STANDARD_REQUIRED ON)

# Options
option(PA_MASTER "Build master-only (exclude slave functions)" OFF)
option(PA_SLAVE  "Build slave-only (exclude master functions)" OFF)
option(PA_NO_FRAMER_TCP "Exclude TCP framer (RTU only)" OFF)
option(PA_NO_FRAMER_RTU "Exclude RTU framer (TCP only)" OFF)
option(PA_BUILD_TESTS   "Build unit tests" ON)

# Collect sources
set(PA_SOURCES
    src/pamodbus.c
    src/pdu.c
    src/crc16.c
)

# Conditionally add framers
if(NOT PA_NO_FRAMER_RTU)
    list(APPEND PA_SOURCES src/framer_rtu.c)
endif()

if(NOT PA_NO_FRAMER_TCP)
    list(APPEND PA_SOURCES src/framer_tcp.c)
endif()

# Create the library
add_library(pamodbus STATIC ${PA_SOURCES})

# Set include path
target_include_directories(pamodbus PUBLIC include)

# Compile definitions
if(PA_MASTER)
    target_compile_definitions(pamodbus PRIVATE PA_MASTER)
endif()

if(PA_SLAVE)
    target_compile_definitions(pamodbus PRIVATE PA_SLAVE)
endif()

if(PA_NO_FRAMER_TCP)
    target_compile_definitions(pamodbus PRIVATE PA_NO_FRAMER_TCP)
endif()

if(PA_NO_FRAMER_RTU)
    target_compile_definitions(pamodbus PRIVATE PA_NO_FRAMER_RTU)
endif()

# Tests
if(PA_BUILD_TESTS)
    enable_testing()

    # Check if a test framework is available
    find_program(CTEST ctest)

    add_executable(test_pamodbus
        test/test_pamodbus.c
        ${PA_SOURCES}
    )
    target_include_directories(test_pamodbus PRIVATE include src)
    target_compile_definitions(test_pamodbus PRIVATE ${PA_COMPILE_DEFS})

    add_test(NAME pamodbus_tests COMMAND test_pamodbus)
endif()