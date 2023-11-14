message(STATUS "BUILD MCC library")

set(MODULE_MCC mcc)

file(GLOB MCC_SOURCE *.cpp)
#list(REMOVE_ITEM MCC_SOURCE ${CMAKE_CURRENT_SOURCE_DIR}/multiCameraBA.cpp)
file(GLOB MCC_INCLUDE *.h)

# import package
find_package(OpenCV CONFIG REQUIRED)
find_package(Ceres CONFIG REQUIRED)
find_package(Boost COMPONENTS filesystem REQUIRED)
find_package(Sophus REQUIRED)

# Generate Lib
add_library(${MODULE_MCC} ${MCC_SOURCE} ${MCC_INCLUDE})

target_link_libraries(${MODULE_MCC} PUBLIC Ceres::ceres)
target_link_libraries(${MODULE_MCC} PUBLIC Sophus::Sophus)
target_link_libraries(${MODULE_MCC} PUBLIC ${OpenCV_LIBS})
target_link_libraries(${MODULE_MCC} PUBLIC ${Boost_LIBRARIES})

# install
target_include_directories(${MODULE_MCC} PUBLIC 
        $<INSTALL_INTERFACE:include>
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
)

install(TARGETS ${MODULE_MCC} EXPORT mccConfig
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
    INCLUDES DESTINATION include
)

install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/
        DESTINATION include/MCC/
        FILES_MATCHING PATTERN "*.h" 
)

install(EXPORT mccConfig 
    FILE mccTarget.cmake
    NAMESPACE MCC::
    DESTINATION share/cmake/MCC
)

include(CMakePackageConfigHelpers)

configure_package_config_file(
    "${CMAKE_SOURCE_DIR}/cmake/mccConfig.cmake.in"
    "${CMAKE_CURRENT_BINARY_DIR}/mccConfig.cmake"
    INSTALL_DESTINATION lib/cmake/MCC
)

write_basic_package_version_file(
    "${CMAKE_CURRENT_BINARY_DIR}/mccConfigVersion.cmake"
    VERSION ${PROJECT_VERSION}
    COMPATIBILITY AnyNewerVersion
)

install(
    FILES "${CMAKE_CURRENT_BINARY_DIR}/mccConfig.cmake"
          "${CMAKE_CURRENT_BINARY_DIR}/mccConfigVersion.cmake"
    DESTINATION share/cmake/MCC
)

