# Minimum version of CMake required
cmake_minimum_required(VERSION 3.10)

# Project name
project(aircraft_flight_mechanics)

# Create a compile_commands.json file (necessary for clangd LSP in neovim)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Add executable
add_executable(aircraft_flight_mechanics ../main.cpp)

