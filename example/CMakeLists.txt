message(STATUS "Build MCC Example")

set(MODULE_MAIN demo)

include_directories("${CMAKE_SOURCE_DIR}/MCC")

add_executable(${MODULE_MAIN} "app.cpp")

target_link_libraries(${MODULE_MAIN} PRIVATE ${Boost_LIBRARIES})
target_link_libraries(${MODULE_MAIN} PRIVATE mcc)
	