#----------------------------------------------------------------
# Generated CMake target import file for configuration "Release".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "as2_map_server::as2_map_server" for configuration "Release"
set_property(TARGET as2_map_server::as2_map_server APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(as2_map_server::as2_map_server PROPERTIES
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/libas2_map_server.so"
  IMPORTED_SONAME_RELEASE "libas2_map_server.so"
  )

list(APPEND _IMPORT_CHECK_TARGETS as2_map_server::as2_map_server )
list(APPEND _IMPORT_CHECK_FILES_FOR_as2_map_server::as2_map_server "${_IMPORT_PREFIX}/lib/libas2_map_server.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
