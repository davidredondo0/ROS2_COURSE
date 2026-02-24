#----------------------------------------------------------------
# Generated CMake target import file for configuration "Release".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "pendulum_driver::pendulum_driver" for configuration "Release"
set_property(TARGET pendulum_driver::pendulum_driver APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(pendulum_driver::pendulum_driver PROPERTIES
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/libpendulum_driver.so"
  IMPORTED_SONAME_RELEASE "libpendulum_driver.so"
  )

list(APPEND _IMPORT_CHECK_TARGETS pendulum_driver::pendulum_driver )
list(APPEND _IMPORT_CHECK_FILES_FOR_pendulum_driver::pendulum_driver "${_IMPORT_PREFIX}/lib/libpendulum_driver.so" )

# Import target "pendulum_driver::pendulum_driver_exe" for configuration "Release"
set_property(TARGET pendulum_driver::pendulum_driver_exe APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(pendulum_driver::pendulum_driver_exe PROPERTIES
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/pendulum_driver/pendulum_driver_exe"
  )

list(APPEND _IMPORT_CHECK_TARGETS pendulum_driver::pendulum_driver_exe )
list(APPEND _IMPORT_CHECK_FILES_FOR_pendulum_driver::pendulum_driver_exe "${_IMPORT_PREFIX}/lib/pendulum_driver/pendulum_driver_exe" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
