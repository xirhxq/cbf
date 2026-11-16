cmake_minimum_required(VERSION 3.16)
project(cbf2026_numerics LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

option(ENABLE_GUROBI "Enable optional Gurobi backend (default uses bundled header-only PIQP)" OFF)
option(CBF_MINIMUM_350_STOP "Build historical stop policy requiring 350 s" OFF)
option(BUILD_HISTORICAL_VARIANTS "Build historical comparison snapshot executables (requires ENABLE_GUROBI=ON)" OFF)

set(CBF_EXTERNAL_INCLUDE "" CACHE PATH "Optional include root containing Eigen and Boost headers")

find_path(BOOST_INCLUDE_DIR boost/multiprecision/cpp_int.hpp
  HINTS "${CBF_EXTERNAL_INCLUDE}" "$ENV{BOOST_ROOT}/include" /opt/homebrew/include /usr/local/include /usr/include
  REQUIRED)

find_package(Eigen3 3.3 QUIET NO_MODULE)
if(TARGET Eigen3::Eigen)
  get_target_property(EIGEN_INCLUDE_DIR Eigen3::Eigen INTERFACE_INCLUDE_DIRECTORIES)
endif()
if(NOT EIGEN_INCLUDE_DIR)
  find_path(EIGEN_INCLUDE_DIR Eigen/Dense
    HINTS "${CBF_EXTERNAL_INCLUDE}" /opt/homebrew/include /usr/local/include /usr/include
    PATH_SUFFIXES eigen3
    REQUIRED)
endif()

add_executable(cbf2026_native src/native_main.cpp)
if(CBF_MINIMUM_350_STOP)
  target_include_directories(cbf2026_native BEFORE PRIVATE variants/min350)
endif()
target_include_directories(cbf2026_native PRIVATE
  include
  vendor/include
  "${BOOST_INCLUDE_DIR}"
  "${EIGEN_INCLUDE_DIR}")
target_compile_definitions(cbf2026_native PRIVATE PROJECT_ROOT="${CMAKE_CURRENT_SOURCE_DIR}")
if(CMAKE_CXX_COMPILER_ID MATCHES "Clang|GNU")
  target_compile_options(cbf2026_native PRIVATE -O1 -g0 -Wno-deprecated-declarations)
endif()

if(ENABLE_GUROBI)
  set(GUROBI_HOME "$ENV{GUROBI_HOME}" CACHE PATH "Gurobi installation root")
  find_path(GUROBI_INCLUDE_DIR gurobi_c++.h HINTS "${GUROBI_HOME}/include" REQUIRED)
  find_library(GUROBI_CXX_LIBRARY NAMES gurobi_c++ HINTS "${GUROBI_HOME}/lib" REQUIRED)
  find_library(GUROBI_LIBRARY NAMES gurobi130 gurobi125 gurobi120 gurobi110 gurobi100 gurobi HINTS "${GUROBI_HOME}/lib" REQUIRED)
  target_include_directories(cbf2026_native PRIVATE "${GUROBI_INCLUDE_DIR}")
  target_compile_definitions(cbf2026_native PRIVATE ENABLE_GUROBI)
  target_link_libraries(cbf2026_native PRIVATE "${GUROBI_CXX_LIBRARY}" "${GUROBI_LIBRARY}")

  if(BUILD_HISTORICAL_VARIANTS)
    foreach(variant IN ITEMS cano gdms shared-main shared-gdms timing-main timing-margin immediate-margin timing-10 timing-21 immediate-cano)
      add_executable(cbf2026_${variant} "variants/${variant}/native_main.cpp")
      target_include_directories(cbf2026_${variant} BEFORE PRIVATE "variants/${variant}" include vendor/include "${BOOST_INCLUDE_DIR}" "${EIGEN_INCLUDE_DIR}" "${GUROBI_INCLUDE_DIR}")
      target_compile_definitions(cbf2026_${variant} PRIVATE ENABLE_GUROBI PROJECT_ROOT="${CMAKE_CURRENT_SOURCE_DIR}")
      if(CMAKE_CXX_COMPILER_ID MATCHES "Clang|GNU")
        target_compile_options(cbf2026_${variant} PRIVATE -O1 -g0 -Wno-deprecated-declarations)
      endif()
      target_link_libraries(cbf2026_${variant} PRIVATE "${GUROBI_CXX_LIBRARY}" "${GUROBI_LIBRARY}")
    endforeach()
  endif()
endif()
