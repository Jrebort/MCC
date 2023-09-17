cmake_minimum_required(VERSION 3.15)

 add_definitions(-DGLOG_NO_ABBREVIATED_SEVERITIES)

set(MCC_VERSION_MAJOR 1.0)
set(MCC_VERSION_MINOR 0.0)

project (MCC VERSION ${MCC_VERSION_MAJOR})

set(CMAKE_CXX_STANDARD 17)

configure_file(MccConfig.h.in ./MCC/MccConfig.h)

add_subdirectory(MCC)
add_subdirectory(example)

