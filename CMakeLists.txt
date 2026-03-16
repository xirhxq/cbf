cmake_minimum_required(VERSION 3.16)
project(cbf)

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

if (ENABLE_GUROBI)
    message(STATUS "Looking for Gurobi...")
    find_package(GUROBI REQUIRED)
    if(GUROBI_FOUND)
        message(STATUS "Found Gurobi: ${GUROBI_LIBRARY}")
        include_directories(${GUROBI_INCLUDE_DIRS})
        target_compile_definitions(cbf_deps INTERFACE ENABLE_GUROBI)
        target_include_directories(cbf_deps INTERFACE ${GUROBI_INCLUDE_DIRS})
        target_link_libraries(cbf_deps INTERFACE ${GUROBI_CXX_LIBRARY} ${GUROBI_LIBRARY})
    else()
        message(WARNING "Gurobi not found. Disabling ENABLE_GUROBI.")
        set(ENABLE_GUROBI OFF)
    endif()
endif ()

if (ENABLE_HIGHS)
    message(STATUS "Looking for HiGHS...")
    find_package(HIGHS REQUIRED)
    if (HIGHS_FOUND)
        message(STATUS "Found HiGHS: ${HIGHS_LIBRARY}")
        target_compile_definitions(cbf_deps INTERFACE ENABLE_HIGHS)
        target_include_directories(cbf_deps INTERFACE ${HIGHS_DIRS})
        target_link_libraries(cbf_deps INTERFACE highs::highs)
    else ()
        message(WARNING "HiGHS not found. Disabling ENABLE_HIGHS.")
        set(ENABLE_HIGHS OFF)
    endif ()
endif ()

if (ENABLE_OSQP)
    message(STATUS "Looking for OsqpEigen...")
    find_package(OsqpEigen REQUIRED)
    if (OsqpEigen_FOUND)
        message(STATUS "Found OsqpEigen")
        target_compile_definitions(cbf_deps INTERFACE ENABLE_OSQP)
        target_link_libraries(cbf_deps INTERFACE OsqpEigen::OsqpEigen)
    else()
        message(WARNING "OsqpEigen not found. Disabling ENABLE_OSQP.")
        set(ENABLE_OSQP OFF)
    endif()
endif()

target_include_directories(cbf_deps INTERFACE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/external
)

if (ENABLE_GUROBI AND GUROBI_FOUND)
    add_definitions(-DENABLE_GUROBI)
endif ()

if (ENABLE_HIGHS AND HIGHS_FOUND)
    add_definitions(-DENABLE_HIGHS)
endif ()

if (ENABLE_OSQP AND OsqpEigen_FOUND)
    add_definitions(-DENABLE_OSQP)
endif ()

include_directories(include external)
file(GLOB SOURCES "src/*.cpp")
foreach (SOURCE ${SOURCES})
    get_filename_component(EXECUTABLE ${SOURCE} NAME_WE)
    add_executable(${EXECUTABLE} ${SOURCE})
    target_compile_definitions(${EXECUTABLE} PRIVATE PROJECT_ROOT="${CMAKE_SOURCE_DIR}")
    target_link_libraries(${EXECUTABLE} PRIVATE cbf_deps)
endforeach ()

file(GLOB TEST_SOURCES "tests/*.cpp")
foreach (TEST_SOURCE ${TEST_SOURCES})
    get_filename_component(TEST_EXECUTABLE ${TEST_SOURCE} NAME_WE)
    add_executable(${TEST_EXECUTABLE} ${TEST_SOURCE})

    target_link_libraries(${TEST_EXECUTABLE} PRIVATE cbf_deps)
endforeach ()
