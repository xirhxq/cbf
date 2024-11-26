cmake_minimum_required(VERSION 3.21)
project(cbf)

set(HIGHS_DIR /usr/local/lib/cmake/highs)
find_package(HIGHS REQUIRED)

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
    target_link_libraries(${EXECUTABLE} libgurobi_c++.a libgurobi120.dylib highs::highs)
endforeach ()

include(FetchContent)
FetchContent_Declare(
        googletest
        URL https://github.com/google/googletest/archive/refs/tags/release-1.12.1.zip
        DOWNLOAD_EXTRACT_TIMESTAMP true
)
FetchContent_MakeAvailable(googletest)

enable_testing()


file(GLOB SOURCES "tests/*.cpp")
foreach (SOURCE ${SOURCES})
    get_filename_component(EXECUTABLE ${SOURCE} NAME_WE)
    add_executable(${EXECUTABLE} ${SOURCE})
    target_link_libraries(${EXECUTABLE} libgurobi_c++.a libgurobi120.dylib highs::highs gtest gtest_main)
endforeach ()