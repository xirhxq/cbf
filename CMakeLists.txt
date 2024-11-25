cmake_minimum_required(VERSION 3.21)
project(cbf)

set(CMAKE_CXX_STANDARD 17)
link_directories(/Library/gurobi1200/macos_universal2/lib)
include_directories(
        /Library/gurobi1200/macos_universal2/include
        include
        external
)

file(GLOB SOURCES "src/*.cpp")
foreach (SOURCE ${SOURCES})
    get_filename_component(EXECUTABLE ${SOURCE} NAME_WE)
    add_executable(${EXECUTABLE} ${SOURCE})
    target_link_libraries(${EXECUTABLE} libgurobi_c++.a libgurobi120.dylib)
endforeach ()