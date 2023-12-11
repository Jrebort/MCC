message(STATUS "Build MCC Example")

set(MODULE_MAIN demo)

# Boost
find_package(Boost COMPONENTS filesystem REQUIRED)

add_library(MCCLIBRARY INTERFACE)
target_include_directories(MCCLIBRARY INTERFACE ${Boost_INCLUDE_DIRS})
target_link_libraries(MCCLIBRARY INTERFACE ${Boost_LIBRARIES})
target_link_libraries(MCCLIBRARY INTERFACE mcc)

add_executable(${MODULE_MAIN} "app.cpp" "lab.cpp" "lab.h")
add_executable(testMat  "testMat.cpp")

target_link_libraries(${MODULE_MAIN}  MCCLIBRARY)
target_link_libraries(testMat  MCCLIBRARY)


	