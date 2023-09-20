#----------------------------------------------------------------
# Generated CMake target import file for configuration "Debug".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "mcc" for configuration "Debug"
set_property(TARGET mcc APPEND PROPERTY IMPORTED_CONFIGURATIONS DEBUG)
set_target_properties(mcc PROPERTIES
  IMPORTED_LINK_INTERFACE_LANGUAGES_DEBUG "CXX"
  IMPORTED_LOCATION_DEBUG "${_IMPORT_PREFIX}/lib/mcc.lib"
  )

list(APPEND _cmake_import_check_targets mcc )
list(APPEND _cmake_import_check_files_for_mcc "${_IMPORT_PREFIX}/lib/mcc.lib" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
