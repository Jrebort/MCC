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
find_package(Boost COMPONENTS filesystem REQUIRED)

# Sophus
find_package(Sophus REQUIRED)

# monoCalibration
add_executable(${PROJECT_NAME} ${MyCSources})

target_link_libraries(${PROJECT_NAME} PRIVATE Ceres::ceres)
target_link_libraries(${PROJECT_NAME} PRIVATE Sophus::Sophus)
target_link_libraries(${PROJECT_NAME} PRIVATE ${OpenCV_LIBS})
target_link_libraries(${PROJECT_NAME} PRIVATE ${Boost_LIBRARIES})
