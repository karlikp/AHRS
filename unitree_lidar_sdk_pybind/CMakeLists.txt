cmake_minimum_required(VERSION 3.22)
project(unitree_lidar_sdk)

# Ustawienie trybu kompilacji
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++14")
set(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -Wall -g2 -ggdb")
set(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3 -Wall -DNDEBUG") 

# Znajdź Python3 i pybind11
find_package(Python3 REQUIRED COMPONENTS Interpreter Development)
find_package(pybind11 REQUIRED)

# Dodaj katalogi do ścieżek dołączania
include_directories(
    ${Python3_INCLUDE_DIRS}  # Nagłówki Pythona
    ${CMAKE_SOURCE_DIR}/include 
)

# Na rasp zmien ścieżke do arm
link_directories(/home/karol/Desktop/repos/SLAM/unitree_lidar_sdk/lib/aarch64 
)

# Tworzenie targetu dla modułu Python
pybind11_add_module(unitree_lidar_sdk_pybind unitree_lidar_sdk_pybind.cpp)

# Dodaj bibliotekę Python3 do ścieżek linkowania
target_link_libraries(unitree_lidar_sdk_pybind PRIVATE /home/karol/Desktop/repos/SLAM/unitree_lidar_sdk/lib/x86_64 libunitree_lidar_sdk.a)
