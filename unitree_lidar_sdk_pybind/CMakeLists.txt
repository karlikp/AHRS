cmake_minimum_required(VERSION 3.22)
project(unitree_lidar_sdk)

# Set compilation mode
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++14")
set(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -Wall -g2 -ggdb")
set(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3 -Wall -DNDEBUG")
set(CMAKE_PREFIX_PATH "/home/pbl/Documents/SLAM/pbl_env/lib/python3.11/site-packages/pybind11/share/cmake/pybind11") 

find_package(Python3 REQUIRED COMPONENTS Interpreter Development)
find_package(pybind11 REQUIRED)

include_directories(
    ${Python3_INCLUDE_DIRS}
    ${CMAKE_SOURCE_DIR}/include 
)

# Pay attention to correct architecture
link_directories(/home/pbl/Documents/SLAM/unitree_lidar_sdk/lib/aarch64 
)

pybind11_add_module(unitree_lidar_sdk_pybind unitree_lidar_sdk_pybind.cpp)

# Pay attention to correct architecture
target_link_libraries(unitree_lidar_sdk_pybind PRIVATE /home/pbl/Documents/SLAM/unitree_lidar_sdk/lib/aarch64 libunitree_lidar_sdk.a)
