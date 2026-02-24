#----------------------------------------------------------------
# Generated CMake target import file for configuration "Release".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "pendulum_controller::pendulum_controller" for configuration "Release"
set_property(TARGET pendulum_controller::pendulum_controller APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(pendulum_controller::pendulum_controller PROPERTIES
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/libpendulum_controller.so"
  IMPORTED_SONAME_RELEASE "libpendulum_controller.so"
  )

list(APPEND _IMPORT_CHECK_TARGETS pendulum_controller::pendulum_controller )
list(APPEND _IMPORT_CHECK_FILES_FOR_pendulum_controller::pendulum_controller "${_IMPORT_PREFIX}/lib/libpendulum_controller.so" )

# Import target "pendulum_controller::pendulum_controller_exe" for configuration "Release"
set_property(TARGET pendulum_controller::pendulum_controller_exe APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(pendulum_controller::pendulum_controller_exe PROPERTIES
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/pendulum_controller/pendulum_controller_exe"
  )

list(APPEND _IMPORT_CHECK_TARGETS pendulum_controller::pendulum_controller_exe )
list(APPEND _IMPORT_CHECK_FILES_FOR_pendulum_controller::pendulum_controller_exe "${_IMPORT_PREFIX}/lib/pendulum_controller/pendulum_controller_exe" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
