cmake_minimum_required(VERSION 3.16)
project(
  mjcf-creator
  VERSION 1.0.0
  LANGUAGES CXX)

# compile_commands.json generation
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# set colored output force
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fdiagnostics-color=always")

# Set C++17 as the standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Add options for building tests and demo
option(BUILD_TESTS "Build tests" ON)
option(BUILD_DEMO "Build demo program" ON)
option(BUILD_EXAMPLES "Build example programs" ON)

include_directories(src)
add_subdirectory(src)

# Conditionally add tests
if(BUILD_TESTS)
  add_subdirectory(tests)
endif()

# Build demo program
if(BUILD_DEMO)
  add_executable(demo demo.cpp)
  target_link_libraries(demo mjcf)
  target_compile_features(demo PRIVATE cxx_std_17)
endif()

# Build examples
if(BUILD_EXAMPLES)
  add_subdirectory(examples)
endif()
