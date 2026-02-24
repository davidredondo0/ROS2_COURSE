#----------------------------------------------------------------
# Generated CMake target import file for configuration "Release".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "as2_behaviors_motion::as2_behaviors_motion" for configuration "Release"
set_property(TARGET as2_behaviors_motion::as2_behaviors_motion APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(as2_behaviors_motion::as2_behaviors_motion PROPERTIES
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/libas2_behaviors_motion.so"
  IMPORTED_SONAME_RELEASE "libas2_behaviors_motion.so"
  )

list(APPEND _IMPORT_CHECK_TARGETS as2_behaviors_motion::as2_behaviors_motion )
list(APPEND _IMPORT_CHECK_FILES_FOR_as2_behaviors_motion::as2_behaviors_motion "${_IMPORT_PREFIX}/lib/libas2_behaviors_motion.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
