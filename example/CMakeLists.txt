message(STATUS "Build MCC Example")

set(MODULE_MAIN demo)

# Boost
find_package(Boost COMPONENTS filesystem REQUIRED)

add_executable(${MODULE_MAIN} "app.cpp" "lab.cpp" "lab.h")

target_include_directories(${MODULE_MAIN} PRIVATE ${Boost_INCLUDE_DIRS})

target_link_libraries(${MODULE_MAIN} PRIVATE ${Boost_LIBRARIES})
target_link_libraries(${MODULE_MAIN} PRIVATE mcc)

	