cmake_minimum_required(VERSION 3.19)

# ���֧�֣���Ϊ MSVC ���������������ء�
if (POLICY CMP0141)
  cmake_policy(SET CMP0141 NEW)
  set(CMAKE_MSVC_DEBUG_INFORMATION_FORMAT "$<IF:$<AND:$<C_COMPILER_ID:MSVC>,$<CXX_COMPILER_ID:MSVC>>,$<$<CONFIG:Debug,RelWithDebInfo>:EditAndContinue>,$<$<CONFIG:Debug,RelWithDebInfo>:ProgramDatabase>>")
endif()

project (MCC VERSION 1.0)
add_definitions(-DGLOG_NO_ABBREVIATED_SEVERITIES)

configure_file(MccConfig.h.in ./MCC/MccConfig.h)

set(CMAKE_CXX_STANDARD 17)
set(MCC_VERSION_MAJOR 1.0)
set(MCC_VERSION_MINOR 0.0)


add_subdirectory(MCC)
add_subdirectory(example)

install(EXPORT mccConfig DESTINATION share/cmake/MCC)
