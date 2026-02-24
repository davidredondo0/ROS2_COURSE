#----------------------------------------------------------------
# Generated CMake target import file for configuration "Release".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "as2_behavior::as2_behavior" for configuration "Release"
set_property(TARGET as2_behavior::as2_behavior APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(as2_behavior::as2_behavior PROPERTIES
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/libas2_behavior.so"
  IMPORTED_SONAME_RELEASE "libas2_behavior.so"
  )

list(APPEND _IMPORT_CHECK_TARGETS as2_behavior::as2_behavior )
list(APPEND _IMPORT_CHECK_FILES_FOR_as2_behavior::as2_behavior "${_IMPORT_PREFIX}/lib/libas2_behavior.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
