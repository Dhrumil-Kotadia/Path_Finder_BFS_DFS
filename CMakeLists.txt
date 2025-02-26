cmake_minimum_required(VERSION 3.10)

# Set the project name
project(GraphPathFinder)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# Add executable
add_executable(path_finder_executable path_finder.cpp)

# Include directories (if needed)
include_directories(${CMAKE_SOURCE_DIR}/include)

# Link any required libraries (if applicable)
# target_link_libraries(GraphPathFinder <LIBRARIES>)
