cmake_minimum_required(VERSION 3.15)
project (cameraCalibration)

file(GLOB MyCSources *.cpp)
set(CMAKE_CXX_STANDARD 17)

#Ceres
find_package(Ceres CONFIG REQUIRED)
include_directories( ${CERES_INCLUDE_DIRS} )

# OpenCV
find_package(OpenCV CONFIG REQUIRED)
include_directories( ${OpenCV_DIRS} )

# monoCalibration
#add_executable(${PROJECT_NAME} ${MyCSources})
add_executable(${PROJECT_NAME} ceres.cpp)


target_link_libraries(${PROJECT_NAME} PRIVATE Ceres::ceres)
target_link_libraries(${PROJECT_NAME} PRIVATE ${OpenCV_LIBS})
