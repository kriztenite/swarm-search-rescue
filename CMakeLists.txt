cmake_minimum_required(VERSION 4.2)
project(SwarmSim)

# Enforce c++20 standard 
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

#Find pybind11 package
find_package(Python COMPONENTS Interpreter Development REQUIRED)
find_package(pybind11 CONFIG REQUIRED)

# Enable testing
enable_testing()

# Tell compiler where to get header files
include_directories(include)

# Python binding core module
# c++source and the pybind11 wrapper
pybind11_add_module(swarm_core
src/Agent.cpp
src/pybind11_wrapper.cpp
)

# Standard c++ test executable
add_executable(unit_tests
tests/test_agent.cpp
src/Agent.cpp
)

add_test(NAME AgentUnitTests COMMAND unit_tests)


