message(STATUS "BUILD MCC library")

set(MODULE_MCC mcc)

file(GLOB MCC_SOURCE *.cpp)
file(GLOB MCC_INCLUDE *.h)

# OpenCV
find_package(OpenCV CONFIG REQUIRED)

# Ceres
find_package(Ceres CONFIG REQUIRED)

# Boost
find_package(Boost COMPONENTS filesystem REQUIRED)

# Sophus
find_package(Sophus REQUIRED)

# Generate Lib
add_library(${MODULE_MCC} ${MCC_SOURCE})

target_link_libraries(${MODULE_MCC} PRIVATE Ceres::ceres)
target_link_libraries(${MODULE_MCC} PRIVATE Sophus::Sophus)
target_link_libraries(${MODULE_MCC} PUBLIC ${OpenCV_LIBS})
target_link_libraries(${MODULE_MCC} PRIVATE ${Boost_LIBRARIES})

target_include_directories(${MODULE_MCC} PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

# install
#target_include_directories(${MODULE_MCC} PUBLIC 
#        $<INSTALL_INTERFACE:include>
#        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
#)
#
#install(TARGETS ${MODULE_MCC} EXPORT MccConfig
#    LIBRARY DESTINATION lib
#    ARCHIVE DESTINATION lib
#    RUNTIME DESTINATION bin
#    INCLUDES DESTINATION include
#)
#
#install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/
#        DESTINATION include
#        FILES_MATCHING PATTERN "*.h" 
#)
#
#install(EXPORT MccConfig 
#    FILE MccConfig.cmake
#    NAMESPACE MCC::
#    DESTINATION lib/cmake/MyProject
#)
