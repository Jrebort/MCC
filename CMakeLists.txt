cmake_minimum_required(VERSION 3.15)

project (MCC VERSION ${MCC_VERSION_MAJOR})
add_definitions(-DGLOG_NO_ABBREVIATED_SEVERITIES)

configure_file(MccConfig.h.in ./MCC/MccConfig.h)
set(CMAKE_CXX_STANDARD 17)
set(MCC_VERSION_MAJOR 1.0)
set(MCC_VERSION_MINOR 0.0)

add_subdirectory(MCC)
add_subdirectory(example)

install(EXPORT MccConfig DESTINATION share/mcc/cmake)
