cmake_minimum_required(VERSION 3.16)
project(cbf)

include(FeatureSummary)

option(CXX "enable C++ compilation" ON)
if(CXX)
    enable_language(CXX)
endif()

set(CMAKE_CXX_STANDARD 17)

option(ENABLE_GUROBI "Enable Gurobi Optimizer" ON)
option(ENABLE_HIGHS "Enable HiGHS Optimizer" ON)
option(ENABLE_OSQP "Enable OSQP Optimizer" ON)
option(ENABLE_PROXQP "Enable ProxQP Optimizer" ON)
set(GRAND_FINALE_SOLVER_PROFILE "" CACHE STRING
    "Require a GrandFinale solver profile: gurobi or open_source")
set_property(CACHE GRAND_FINALE_SOLVER_PROFILE PROPERTY STRINGS
    "" gurobi open_source)

set(CBF_EXTERNAL_INCLUDE_DIRS
    ${CMAKE_CURRENT_SOURCE_DIR}/external
    ${CMAKE_CURRENT_SOURCE_DIR}/external/doctest
    ${CMAKE_CURRENT_SOURCE_DIR}/external/nlohmann
)

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

if(GRAND_FINALE_SOLVER_PROFILE STREQUAL "gurobi")
    if(NOT ENABLE_GUROBI)
        message(FATAL_ERROR
            "GrandFinale gurobi profile requires Gurobi")
    endif()
elseif(GRAND_FINALE_SOLVER_PROFILE STREQUAL "open_source")
    if(NOT ENABLE_HIGHS OR NOT ENABLE_OSQP)
        message(FATAL_ERROR
            "GrandFinale open_source profile requires HiGHS and OSQP")
    endif()
elseif(NOT GRAND_FINALE_SOLVER_PROFILE STREQUAL "")
    message(FATAL_ERROR
        "GRAND_FINALE_SOLVER_PROFILE must be gurobi or open_source")
endif()

set_package_properties(proxsuite PROPERTIES
    DESCRIPTION "ProxSuite ProxQP QP solver"
    TYPE RECOMMENDED)

if(ENABLE_PROXQP)
    message(STATUS "Looking for ProxSuite...")
    find_package(proxsuite)
    if(proxsuite_FOUND)
        message(STATUS "Found ProxSuite")
        if(TARGET Eigen3::Eigen)
            get_target_property(PROXQP_EIGEN_INCLUDE_DIRS Eigen3::Eigen INTERFACE_INCLUDE_DIRECTORIES)
            include_directories(BEFORE ${PROXQP_EIGEN_INCLUDE_DIRS})
            target_include_directories(cbf_deps BEFORE INTERFACE ${PROXQP_EIGEN_INCLUDE_DIRS})
        endif()
        set(CBF_EXTERNAL_INCLUDE_DIRS
            ${CMAKE_CURRENT_SOURCE_DIR}/external/doctest
            ${CMAKE_CURRENT_SOURCE_DIR}/external/nlohmann
        )
        target_compile_definitions(cbf_deps INTERFACE ENABLE_PROXQP)
        include_directories(BEFORE ${proxsuite_INCLUDE_DIRS})
        target_include_directories(cbf_deps BEFORE INTERFACE ${proxsuite_INCLUDE_DIRS})
    else()
        message(STATUS "ProxSuite not found - disabled")
        set(ENABLE_PROXQP OFF CACHE BOOL "Enable ProxQP Optimizer" FORCE)
    endif()
endif()

target_include_directories(cbf_deps INTERFACE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CBF_EXTERNAL_INCLUDE_DIRS}
)

include_directories(include ${CBF_EXTERNAL_INCLUDE_DIRS})

file(GLOB SOURCES "src/*.cpp")
foreach(SOURCE ${SOURCES})
    get_filename_component(EXECUTABLE ${SOURCE} NAME_WE)
    add_executable(${EXECUTABLE} ${SOURCE})
    target_compile_definitions(${EXECUTABLE} PRIVATE PROJECT_ROOT="${CMAKE_SOURCE_DIR}")
    target_link_libraries(${EXECUTABLE} PRIVATE cbf_deps)
endforeach()

file(GLOB_RECURSE CBF_BUILD_FINGERPRINT_FILES
    CONFIGURE_DEPENDS
    RELATIVE "${CMAKE_CURRENT_SOURCE_DIR}"
    "${CMAKE_CURRENT_SOURCE_DIR}/CMakeLists.txt"
    "${CMAKE_CURRENT_SOURCE_DIR}/cmake/*.cmake"
    "${CMAKE_CURRENT_SOURCE_DIR}/external/*"
    "${CMAKE_CURRENT_SOURCE_DIR}/include/*"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/*"
)
list(SORT CBF_BUILD_FINGERPRINT_FILES)
set(CBF_BUILD_FINGERPRINT_INPUT "")
set(CBF_BUILD_FINGERPRINT_ABSOLUTE_FILES "")
foreach(RELATIVE_PATH ${CBF_BUILD_FINGERPRINT_FILES})
    if(NOT IS_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/${RELATIVE_PATH}")
        list(APPEND CBF_BUILD_FINGERPRINT_ABSOLUTE_FILES
            "${CMAKE_CURRENT_SOURCE_DIR}/${RELATIVE_PATH}")
        file(SHA256
            "${CMAKE_CURRENT_SOURCE_DIR}/${RELATIVE_PATH}"
            FILE_SHA256)
        string(APPEND CBF_BUILD_FINGERPRINT_INPUT
            "${RELATIVE_PATH}:${FILE_SHA256}\n")
    endif()
endforeach()
set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS
    ${CBF_BUILD_FINGERPRINT_ABSOLUTE_FILES})
string(SHA256 CBF_BUILD_SOURCE_SHA256 "${CBF_BUILD_FINGERPRINT_INPUT}")
if(TARGET Swarm)
    target_compile_definitions(
        Swarm PRIVATE CBF_BUILD_SOURCE_SHA256="${CBF_BUILD_SOURCE_SHA256}")
endif()

file(GLOB TEST_SOURCES CONFIGURE_DEPENDS "tests/*.cpp")
foreach(TEST_SOURCE ${TEST_SOURCES})
    get_filename_component(TEST_EXECUTABLE ${TEST_SOURCE} NAME_WE)
    add_executable(${TEST_EXECUTABLE} ${TEST_SOURCE})
    target_compile_definitions(
        ${TEST_EXECUTABLE} PRIVATE PROJECT_ROOT="${CMAKE_SOURCE_DIR}")
    target_link_libraries(${TEST_EXECUTABLE} PRIVATE cbf_deps)
endforeach()

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
if(ENABLE_PROXQP)
    message(STATUS "  ProxQP: ENABLED")
else()
    message(STATUS "  ProxQP: disabled (not found)")
endif()
message(STATUS "")

if(NOT ENABLE_GUROBI AND NOT ENABLE_HIGHS AND NOT ENABLE_OSQP AND NOT ENABLE_PROXQP)
    message(WARNING "
===============================================================================
WARNING: No optimizer found! Please install at least one of:
  - Gurobi (commercial): https://www.gurobi.com/
  - HiGHS (open-source): brew install highs
  - OSQP (open-source): brew install osqp && install osqp-eigen
  - ProxQP (open-source): brew install proxsuite
===============================================================================
")
endif()
