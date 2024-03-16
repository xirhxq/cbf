cmake_minimum_required(VERSION 3.21)
project(cbf_main)

set(CMAKE_CXX_STANDARD 14)
link_directories(/Library/gurobi1000/macos_universal2/lib)
include_directories(/Library/gurobi1000/macos_universal2/include)
include_directories(include)

add_executable(cbf_main src/cbf_main.cpp)
target_link_libraries(cbf_main libgurobi_c++.a libgurobi100.dylib)