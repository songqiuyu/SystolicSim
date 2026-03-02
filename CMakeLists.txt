cmake_minimum_required(VERSION 3.14)
project(SystolicSimSC CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ===========================================================================
# Options
# ===========================================================================

# Memory backend: NONE | SIMPLE | RAMULATOR
set(MEM_BACKEND "SIMPLE" CACHE STRING "Memory backend: NONE / SIMPLE / RAMULATOR")
set_property(CACHE MEM_BACKEND PROPERTY STRINGS NONE SIMPLE RAMULATOR)

message(STATUS "=== Systolic Array SystemC Simulator ===")
message(STATUS "  MEM_BACKEND: ${MEM_BACKEND}")

# ===========================================================================
# Find SystemC
# ===========================================================================
find_package(SystemCLanguage QUIET)

if(SystemCLanguage_FOUND)
    message(STATUS "  SystemC: found via SystemCLanguage package")
    set(SYSTEMC_FOUND TRUE)
else()
    # Fallback: look for SYSTEMC_HOME env var
    if(DEFINED ENV{SYSTEMC_HOME})
        set(SYSTEMC_HOME $ENV{SYSTEMC_HOME})
        message(STATUS "  SystemC: using SYSTEMC_HOME=${SYSTEMC_HOME}")
        find_library(SYSTEMC_LIB NAMES systemc PATHS "${SYSTEMC_HOME}/lib" "${SYSTEMC_HOME}/lib-linux64"
                                                      "${SYSTEMC_HOME}/lib-mingw64" "${SYSTEMC_HOME}/lib-msvc-14.3")
        set(SYSTEMC_INCLUDE_DIRS "${SYSTEMC_HOME}/include")
        set(SYSTEMC_FOUND TRUE)
    else()
        message(WARNING "  SystemC NOT found. Set SYSTEMC_HOME or install SystemCLanguage CMake package.")
        set(SYSTEMC_FOUND FALSE)
    endif()
endif()

# ===========================================================================
# Ramulator 2.0 path (only needed for RAMULATOR backend)
# ===========================================================================
set(RAMULATOR2_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../ramulator2" CACHE PATH "Path to Ramulator 2.0")

# ===========================================================================
# Source files
# ===========================================================================
set(SC_SOURCES
    src_sc/MemoryInterface.cpp
    src_sc/Scheduler.cpp
    src_sc/sc_main.cpp
)

set(SC_HEADERS
    src_sc/Config.h
    src_sc/Stats.h
    src_sc/PE.h
    src_sc/SystolicArray.h
    src_sc/Buffer.h
    src_sc/MemoryInterface.h
    src_sc/Scheduler.h
    src_sc/Top.h
)

# ===========================================================================
# Executable
# ===========================================================================
add_executable(systolic_sc ${SC_SOURCES} ${SC_HEADERS})

# Include directories
target_include_directories(systolic_sc PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src_sc
)

# Compile definition for memory backend
string(TOUPPER "${MEM_BACKEND}" MEM_BACKEND_UPPER)
target_compile_definitions(systolic_sc PRIVATE MEM_BACKEND_${MEM_BACKEND_UPPER}=1)

if(MEM_BACKEND_UPPER STREQUAL "RAMULATOR")
    target_compile_definitions(systolic_sc PRIVATE USE_RAMULATOR=1)
endif()

# ===========================================================================
# Link SystemC
# ===========================================================================
if(SystemCLanguage_FOUND)
    target_link_libraries(systolic_sc SystemC::systemc)
elseif(SYSTEMC_FOUND AND SYSTEMC_LIB)
    target_include_directories(systolic_sc PRIVATE ${SYSTEMC_INCLUDE_DIRS})
    target_link_libraries(systolic_sc ${SYSTEMC_LIB})
else()
    message(FATAL_ERROR "SystemC library not found. Cannot build systolic_sc.")
endif()

# ===========================================================================
# Ramulator (conditional)
# ===========================================================================
if(MEM_BACKEND_UPPER STREQUAL "RAMULATOR")
    if(EXISTS "${RAMULATOR2_DIR}/src")
        target_include_directories(systolic_sc PRIVATE
            ${RAMULATOR2_DIR}/src
            ${RAMULATOR2_DIR}/ext/spdlog/include
            ${RAMULATOR2_DIR}/ext/yaml-cpp/include
            ${RAMULATOR2_DIR}/ext/argparse/include
        )
        target_link_directories(systolic_sc PRIVATE ${RAMULATOR2_DIR}/build)
        target_link_libraries(systolic_sc ramulator yaml-cpp)
        message(STATUS "  Ramulator2: ${RAMULATOR2_DIR}")
    else()
        message(FATAL_ERROR "MEM_BACKEND=RAMULATOR but Ramulator2 not found at ${RAMULATOR2_DIR}")
    endif()
endif()

# ===========================================================================
# Config copy
# ===========================================================================
configure_file(
    ${CMAKE_CURRENT_SOURCE_DIR}/config/config_ddr5_4ch.yaml
    ${CMAKE_CURRENT_BINARY_DIR}/config/config_ddr5_4ch.yaml
    COPYONLY
)
configure_file(
    ${CMAKE_CURRENT_SOURCE_DIR}/config/config_lpddr4_2400.yaml
    ${CMAKE_CURRENT_BINARY_DIR}/config/config_lpddr4_2400.yaml
    COPYONLY
)

# ===========================================================================
# Build info
# ===========================================================================
message(STATUS "  Build type: ${CMAKE_BUILD_TYPE}")
message(STATUS "  C++ standard: ${CMAKE_CXX_STANDARD}")
message(STATUS "========================================")
