cmake_minimum_required(VERSION 3.14)
project(unitree_lidar_sdk_pybind)

# Ustawienia wersji języka C++
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Dodanie Pybind11
find_package(pybind11 REQUIRED)

# Dodanie libpointmatcher
find_package(libpointmatcher REQUIRED)

# Znalezienie Eigen3
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})


# Ścieżka do Unitree SDK (dostosuj do lokalizacji plików SDK)
set(UNITREE_SDK_PATH "${CMAKE_SOURCE_DIR}/../unitree_lidar_sdk")
link_directories(${UNITREE_SDK_PATH}/lib/aarch64)
#include_directories(${UNITREE_SDK_PATH}/lib/aarch64/libunitree_lidar_sdk)

# Ścieżki do libpointmatcher (używane domyślne lokalizacje)
include_directories(${LIBPOINTMATCHER_INCLUDE_DIRS})
link_directories(${LIBPOINTMATCHER_LIBRARY_DIRS})

# Dodanie plików źródłowych
set(SOURCES
    unitree_lidar_sdk_pybind.cpp
    function.cpp
)

# Tworzenie targetu pybind11 jako modułu Python
pybind11_add_module(unitree_lidar_sdk_pybind ${SOURCES})

# Dodanie bibliotek do linkowania
target_link_libraries(unitree_lidar_sdk_pybind
    PRIVATE
    pybind11::module
    ${LIBPOINTMATCHER_LIBRARIES}
    unitree_lidar_sdk
)

# Dodanie ścieżek do zależności
target_include_directories(unitree_lidar_sdk_pybind PRIVATE ${CMAKE_SOURCE_DIR})
target_include_directories(unitree_lidar_sdk_pybind PRIVATE ${UNITREE_SDK_PATH}/include)

# Opcjonalne: ustawienia kompilatora
target_compile_options(unitree_lidar_sdk_pybind PRIVATE -Wall -Wextra -Wpedantic)
