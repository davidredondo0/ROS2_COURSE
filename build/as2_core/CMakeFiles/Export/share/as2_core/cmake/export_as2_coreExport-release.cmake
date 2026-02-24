#----------------------------------------------------------------
# Generated CMake target import file for configuration "Release".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "as2_core::as2_core" for configuration "Release"
set_property(TARGET as2_core::as2_core APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(as2_core::as2_core PROPERTIES
  IMPORTED_LINK_INTERFACE_LANGUAGES_RELEASE "CXX"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/libas2_core.a"
  )

list(APPEND _IMPORT_CHECK_TARGETS as2_core::as2_core )
list(APPEND _IMPORT_CHECK_FILES_FOR_as2_core::as2_core "${_IMPORT_PREFIX}/lib/libas2_core.a" )

# Import target "as2_core::mocks" for configuration "Release"
set_property(TARGET as2_core::mocks APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(as2_core::mocks PROPERTIES
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/libmocks.so"
  IMPORTED_SONAME_RELEASE "libmocks.so"
  )

list(APPEND _IMPORT_CHECK_TARGETS as2_core::mocks )
list(APPEND _IMPORT_CHECK_FILES_FOR_as2_core::mocks "${_IMPORT_PREFIX}/lib/libmocks.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
