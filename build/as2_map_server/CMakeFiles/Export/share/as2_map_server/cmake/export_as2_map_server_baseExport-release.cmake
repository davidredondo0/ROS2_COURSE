#----------------------------------------------------------------
# Generated CMake target import file for configuration "Release".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "as2_map_server::as2_map_server_base" for configuration "Release"
set_property(TARGET as2_map_server::as2_map_server_base APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(as2_map_server::as2_map_server_base PROPERTIES
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/libas2_map_server_base.so"
  IMPORTED_SONAME_RELEASE "libas2_map_server_base.so"
  )

list(APPEND _IMPORT_CHECK_TARGETS as2_map_server::as2_map_server_base )
list(APPEND _IMPORT_CHECK_FILES_FOR_as2_map_server::as2_map_server_base "${_IMPORT_PREFIX}/lib/libas2_map_server_base.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
