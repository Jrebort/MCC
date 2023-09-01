cmake_minimum_required(VERSION 3.15)
project (cameraCalibration)

file(GLOB MyCSources *.cpp)
set(CMAKE_CXX_STANDARD 17)

# OpenCV
find_package(OpenCV CONFIG REQUIRED)
include_directories( ${OpenCV_DIRS} )

# Ceres
find_package(Ceres CONFIG REQUIRED)
include_directories( ${CERES_INCLUDE_DIRS} )

# Boost
set(Boost_USE_STATIC_LIBS ON)
set(Boost_USE_STATIC_RUNTIME ON)
set(Boost_INCLUDE_DIR ${_VCPKG_INSTALLED_DIR}/x64-windows-static/include)
set_property(GLOBAL PROPERTY USE_FOLDERS ON)
find_package(Boost COMPONENTS filesystem REQUIRED)
if (Boost_FOUND)
    include_directories(${Boost_INCLUDE_DIRS})
    link_directories(${Boost_LIBRARY_DIRS})
endif ()


# monoCalibration
add_executable(${PROJECT_NAME} ${MyCSources} Dataset.cpp)
#add_executable(${PROJECT_NAME} multiCameraCalibration.cpp)


target_link_libraries(${PROJECT_NAME} PRIVATE Ceres::ceres)
target_link_libraries(${PROJECT_NAME} PRIVATE ${OpenCV_LIBS})
target_link_libraries(${PROJECT_NAME} PRIVATE ${Boost_LIBRARIES})
