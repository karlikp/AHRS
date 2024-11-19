cmake_minimum_required(VERSION 3.0)
project(unitree_lidar_sdk)

# Set CMake build type
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++14")
set(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -Wall -g2 -ggdb")
set(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3 -Wall -DNDEBUG") 

# Include directories for header files
include_directories(include)

# Link directories for libraries
link_directories(lib/${CMAKE_SYSTEM_PROCESSOR})

# Set output directories for executables and libraries
SET(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib/${CMAKE_SYSTEM_PROCESSOR})
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib/${CMAKE_SYSTEM_PROCESSOR})

# Create shared library
add_library(unitree_lidar_sdk SHARED
  src/unitree_lidar_sdk.cpp  # Add your source files here, if needed
  # Add other .cpp files related to the SDK if needed
)

# If there are any specific dependencies required for your shared library, link them here
# target_link_libraries(unitree_lidar_sdk <dependencies>)

# Create executables
add_executable(example_lidar
  examples/example_lidar.cpp
)
target_link_libraries(example_lidar unitree_lidar_sdk)

add_executable(example_lidar_udp
  examples/example_lidar_udp.cpp
)
target_link_libraries(example_lidar_udp unitree_lidar_sdk)

add_executable(unilidar_publisher_udp
  examples/unilidar_publisher_udp.cpp
)
target_link_libraries(unilidar_publisher_udp unitree_lidar_sdk)

add_executable(unilidar_subscriber_udp
  examples/unilidar_subscriber_udp.cpp
)
target_link_libraries(unilidar_subscriber_udp unitree_lidar_sdk)

