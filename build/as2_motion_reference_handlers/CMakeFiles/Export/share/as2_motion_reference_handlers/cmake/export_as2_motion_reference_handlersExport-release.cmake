#----------------------------------------------------------------
# Generated CMake target import file for configuration "Release".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "as2_motion_reference_handlers::as2_motion_reference_handlers" for configuration "Release"
set_property(TARGET as2_motion_reference_handlers::as2_motion_reference_handlers APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(as2_motion_reference_handlers::as2_motion_reference_handlers PROPERTIES
  IMPORTED_LINK_INTERFACE_LANGUAGES_RELEASE "CXX"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/libas2_motion_reference_handlers.a"
  )

list(APPEND _IMPORT_CHECK_TARGETS as2_motion_reference_handlers::as2_motion_reference_handlers )
list(APPEND _IMPORT_CHECK_FILES_FOR_as2_motion_reference_handlers::as2_motion_reference_handlers "${_IMPORT_PREFIX}/lib/libas2_motion_reference_handlers.a" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
