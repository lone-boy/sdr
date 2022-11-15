cmake_minimum_required(VERSION 3.22)
project(sdr)

set(CMAKE_CXX_STANDARD 14)

include_directories(inc)

add_executable(sdr main.cpp inc/sdrDevice.h
        inc/ad9361.h
        inc/iio.h
        src/sdrDevice.cpp inc/log.h src/log.cpp)

target_link_libraries(sdr iio ad9361 uhd pthread)
