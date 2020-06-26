cmake_minimum_required(VERSION 3.1)

project(TSDFFuion)

find_package(OpenCV REQUIRED)
add_executable(tsdf-fusion-cpu tsdf_fusion_cpu.cpp)
target_link_libraries(tsdf-fusion-cpu ${OpenCV_LIBS})

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED True)
