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

# Znajdź bibliotekę yaml-cpp
find_package(yaml-cpp REQUIRED)

# Znalezienie libnabo
find_library(LIBNABO_LIBRARY nabo REQUIRED PATHS /usr/local/lib /usr/lib)
find_path(LIBNABO_INCLUDE_DIRS "nabo/nabo.h" PATHS /usr/local/include /usr/include)

# Znalezienie OpenMP
find_package(OpenMP REQUIRED)

# Sprawdzenie, czy znaleziono libnabo
if (NOT LIBNABO_LIBRARY OR NOT LIBNABO_INCLUDE_DIRS)
    message(FATAL_ERROR "libnabo not found!")
endif()


# Ścieżka do Unitree SDK (dostosuj do lokalizacji plików SDK)
set(UNITREE_SDK_PATH "${CMAKE_SOURCE_DIR}/../unitree_lidar_sdk")
link_directories(${UNITREE_SDK_PATH}/lib/aarch64)
#include_directories(${UNITREE_SDK_PATH}/lib/aarch64/libunitree_lidar_sdk)

# Dodanie ścieżek do nagłówków i bibliotek statycznych dla libpointmatcher
include_directories(/usr/local/include/pointmatcher)
set(LIBPOINTMATCHER_STATIC_LIB /usr/local/lib/libpointmatcher.a)

# Dodanie plików źródłowych
set(SOURCES
    unitree_lidar_sdk_pybind.cpp
    function.cpp
)

# Tworzenie targetu pybind11 jako modułu Python
pybind11_add_module(unitree_lidar_sdk_pybind ${SOURCES})

# Dodaj flagę -fopenmp do opcji kompilacji
target_compile_options(unitree_lidar_sdk_pybind PRIVATE -fopenmp)
target_link_libraries(unitree_lidar_sdk_pybind PRIVATE -fopenmp)


# Dodanie bibliotek do linkowania
target_link_libraries(unitree_lidar_sdk_pybind
    PRIVATE
    pybind11::module
    ${LIBPOINTMATCHER_STATIC_LIB}
    ${LIBNABO_LIBRARY}
    unitree_lidar_sdk
    yaml-cpp
    lz4
    /usr/lib/gcc/aarch64-linux-gnu/12/libgomp.so

)

# Dodanie ścieżek do nagłówków
target_include_directories(unitree_lidar_sdk_pybind PRIVATE 
    ${CMAKE_SOURCE_DIR}
    ${UNITREE_SDK_PATH}/include
    ${LIBNABO_INCLUDE_DIRS}    # Dodanie ścieżek nagłówków libnabo
)

# Opcjonalne: ustawienia kompilatora
target_compile_options(unitree_lidar_sdk_pybind PRIVATE -Wall -Wextra -Wpedantic)
