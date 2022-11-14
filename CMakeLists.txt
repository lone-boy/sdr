cmake_minimum_required(VERSION 3.22)
project(sdr)

set(CMAKE_CXX_STANDARD 14)

include_directories(inc)

add_executable(sdr main.cpp inc/sdrDevice.h src/sdrDevice.cpp)
