cmake_minimum_required(VERSION 3.14)
project(unitree_lidar_sdk_pybind)

# Ustawienia wersji języka C++
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Dodanie Pybind11
find_package(pybind11 REQUIRED)

# Dodanie libpointmatcher
find_package(libpointmatcher REQUIRED)

# Ścieżka do Unitree SDK (zaktualizuj ją zgodnie z lokalizacją)
set(UNITREE_SDK_PATH "../unitree_lidar_sdk")
include_directories(${UNITREE_SDK_PATH}/include)

# Ścieżki do libpointmatcher
set(LIBPOINTMATCHER_STATIC_PATH "/usr/local/lib/libpointmatcher.a")
include_directories("/usr/local/include")  # Domyślne ścieżki dla nagłówków

# Ścieżka do libpointmatcher (jeśli biblioteka jest w nietypowej lokalizacji, dostosuj)
include_directories(${LIBPOINTMATCHER_INCLUDE_DIRS})

# Dodanie plików źródłowych
set(SOURCES
    unitree_lidar_sdk_pybind.cpp
    function.cpp
)

# Tworzenie targetu biblioteki
pybind11_add_module(unitree_lidar_sdk_pybind ${SOURCES})

# Dodanie bibliotek do linkowania
target_link_libraries(unitree_lidar_sdk_pybind
    ${LIBPOINTMATCHER_LIBRARIES}
    ${UNITREE_SDK_PATH}/lib/libunitree_lidar_sdk.a
)

# Dodanie ścieżek do zależności
target_include_directories(unitree_lidar_sdk_pybind PRIVATE ${CMAKE_SOURCE_DIR})
target_include_directories(unitree_lidar_sdk_pybind PRIVATE ${LIBPOINTMATCHER_INCLUDE_DIRS})
target_include_directories(unitree_lidar_sdk_pybind PRIVATE ${UNITREE_SDK_PATH}/include)

# Opcjonalne: ustawienia kompilatora
target_compile_options(unitree_lidar_sdk_pybind PRIVATE -Wall -Wextra -Wpedantic)
