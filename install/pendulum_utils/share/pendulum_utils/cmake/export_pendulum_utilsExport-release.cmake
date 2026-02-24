#----------------------------------------------------------------
# Generated CMake target import file for configuration "Release".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "pendulum_utils::pendulum_utils" for configuration "Release"
set_property(TARGET pendulum_utils::pendulum_utils APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(pendulum_utils::pendulum_utils PROPERTIES
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/libpendulum_utils.so"
  IMPORTED_SONAME_RELEASE "libpendulum_utils.so"
  )

list(APPEND _IMPORT_CHECK_TARGETS pendulum_utils::pendulum_utils )
list(APPEND _IMPORT_CHECK_FILES_FOR_pendulum_utils::pendulum_utils "${_IMPORT_PREFIX}/lib/libpendulum_utils.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
