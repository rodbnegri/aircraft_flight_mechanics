# Minimum version of CMake required
cmake_minimum_required(VERSION 3.10)

# Project name
project(aircraft_flight_mechanics)

# Add the include directory to the include path
include_directories(include)

# Add all source files in the src directory to the SOURCES variable
file(GLOB SOURCES "src/*.cpp")

# Add an executable target using the source files
add_executable(main ${SOURCES})

# Enable testing
enable_testing()

# Add a test executable target named "my_test" using the test source files
add_executable(test_atmosphere tests/test_atmosphere.cpp)

# Add a test called "run_my_test" that runs the "my_test" executable
add_test(NAME test_atmosphere COMMAND test_atmosphere)

# Create a compile_commands.json file (necessary for clangd LSP in neovim)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

