cmake_minimum_required(VERSION 3.21)
project(cbf)

set(CMAKE_CXX_STANDARD 14)
link_directories(/Library/gurobi1000/macos_universal2/lib)
include_directories(/Library/gurobi1000/macos_universal2/include)
include_directories(include)

add_executable(main src/main.cpp)
target_link_libraries(main libgurobi_c++.a libgurobi100.dylib)