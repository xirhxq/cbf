cmake_minimum_required(VERSION 3.21)
project(cbf_main)

set(CMAKE_CXX_STANDARD 14)
link_directories(/Library/gurobi951/macos_universal2/lib)
include_directories(/Library/gurobi951/macos_universal2/include)

add_executable(cbf_main cbf_main.cpp Robot.cpp World.cpp
        computing_geometry/Point.cpp computing_geometry/Line.cpp
        computing_geometry/Polygon.cpp computing_geometry/CVT.cpp
        Swarm.cpp CBF.cpp Target.cpp GridWorld.cpp)
target_link_libraries(cbf_main libgurobi_c++.a libgurobi95.dylib)