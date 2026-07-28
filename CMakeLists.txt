cmake_minimum_required(VERSION 3.16)
project(cbf)

include(FeatureSummary)
include(CTest)

option(CXX "enable C++ compilation" ON)
if(CXX)
    enable_language(CXX)
endif()

set(CMAKE_CXX_STANDARD 17)

option(ENABLE_GUROBI "Enable Gurobi Optimizer" ON)
option(ENABLE_HIGHS "Enable HiGHS Optimizer" ON)
option(ENABLE_OSQP "Enable OSQP Optimizer" ON)

list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release")
endif()
message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")

add_library(cbf_deps INTERFACE)

set_package_properties(GUROBI PROPERTIES
    DESCRIPTION "Commercial high-performance optimizer"
    TYPE RECOMMENDED)

if(ENABLE_GUROBI)
    message(STATUS "Looking for Gurobi...")
    find_package(GUROBI)
    if(GUROBI_FOUND)
        message(STATUS "Found Gurobi: ${GUROBI_LIBRARY}")
        target_compile_definitions(cbf_deps INTERFACE ENABLE_GUROBI)
        target_include_directories(cbf_deps INTERFACE ${GUROBI_INCLUDE_DIRS})
        target_link_libraries(cbf_deps INTERFACE ${GUROBI_CXX_LIBRARY} ${GUROBI_LIBRARY})
    else()
        message(STATUS "Gurobi not found - disabled")
        set(ENABLE_GUROBI OFF CACHE BOOL "Enable Gurobi Optimizer" FORCE)
    endif()
endif()

set_package_properties(HIGHS PROPERTIES
    DESCRIPTION "Open-source linear optimization solver"
    TYPE RECOMMENDED)

if(ENABLE_HIGHS)
    message(STATUS "Looking for HiGHS...")
    find_package(HIGHS)
    if(HIGHS_FOUND)
        message(STATUS "Found HiGHS: ${HIGHS_LIBRARY}")
        target_compile_definitions(cbf_deps INTERFACE ENABLE_HIGHS)
        target_include_directories(cbf_deps INTERFACE ${HIGHS_DIRS})
        target_link_libraries(cbf_deps INTERFACE highs::highs)
    else()
        message(STATUS "HiGHS not found - disabled")
        set(ENABLE_HIGHS OFF CACHE BOOL "Enable HiGHS Optimizer" FORCE)
    endif()
endif()

set_package_properties(OsqpEigen PROPERTIES
    DESCRIPTION "OSQP QP solver with Eigen wrapper"
    TYPE RECOMMENDED)

if(ENABLE_OSQP)
    message(STATUS "Looking for OsqpEigen...")
    find_package(OsqpEigen)
    if(OsqpEigen_FOUND)
        message(STATUS "Found OsqpEigen")
        target_compile_definitions(cbf_deps INTERFACE ENABLE_OSQP)
        target_link_libraries(cbf_deps INTERFACE OsqpEigen::OsqpEigen)
    else()
        message(STATUS "OsqpEigen not found - disabled")
        set(ENABLE_OSQP OFF CACHE BOOL "Enable OSQP Optimizer" FORCE)
    endif()
endif()

target_include_directories(cbf_deps INTERFACE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/external
)

include_directories(include external)

file(GLOB SOURCES "src/*.cpp")
foreach(SOURCE ${SOURCES})
    get_filename_component(EXECUTABLE ${SOURCE} NAME_WE)
    add_executable(${EXECUTABLE} ${SOURCE})
    target_compile_definitions(${EXECUTABLE} PRIVATE PROJECT_ROOT="${CMAKE_SOURCE_DIR}")
    target_link_libraries(${EXECUTABLE} PRIVATE cbf_deps)
endforeach()

file(GLOB TEST_SOURCES "tests/*.cpp")
foreach(TEST_SOURCE ${TEST_SOURCES})
    get_filename_component(TEST_EXECUTABLE ${TEST_SOURCE} NAME_WE)
    add_executable(${TEST_EXECUTABLE} ${TEST_SOURCE})
    target_link_libraries(${TEST_EXECUTABLE} PRIVATE cbf_deps)
endforeach()

if(BUILD_TESTING)
    find_package(Python3 COMPONENTS Interpreter)
    if(Python3_Interpreter_FOUND)
        add_test(
            NAME testSwarmFailureExit
            COMMAND
                ${CMAKE_COMMAND} -E env
                CBF_SWARM_BINARY=$<TARGET_FILE:Swarm>
                ${Python3_EXECUTABLE} -m unittest
                tests.test_swarm_failure_exit -v
        )
        set_tests_properties(
            testSwarmFailureExit
            PROPERTIES WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
        )
    endif()
endif()

message(STATUS "")
message(STATUS "=== Optimizer Summary ===")
if(ENABLE_GUROBI)
    message(STATUS "  Gurobi: ENABLED")
else()
    message(STATUS "  Gurobi: disabled (not found)")
endif()
if(ENABLE_HIGHS)
    message(STATUS "  HiGHS:  ENABLED")
else()
    message(STATUS "  HiGHS:  disabled (not found)")
endif()
if(ENABLE_OSQP)
    message(STATUS "  OSQP:   ENABLED")
else()
    message(STATUS "  OSQP:   disabled (not found)")
endif()
message(STATUS "")

if(NOT ENABLE_GUROBI AND NOT ENABLE_HIGHS AND NOT ENABLE_OSQP)
    message(WARNING "
===============================================================================
WARNING: No optimizer found! Please install at least one of:
  - Gurobi (commercial): https://www.gurobi.com/
  - HiGHS (open-source): brew install highs
  - OSQP (open-source): brew install osqp && install osqp-eigen
===============================================================================
")
endif()
