#----------------------------------------------------------------
# Generated CMake target import file for configuration "Release".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "objectpose_msgs::objectpose_msgs__rosidl_typesupport_introspection_cpp" for configuration "Release"
set_property(TARGET objectpose_msgs::objectpose_msgs__rosidl_typesupport_introspection_cpp APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(objectpose_msgs::objectpose_msgs__rosidl_typesupport_introspection_cpp PROPERTIES
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/libobjectpose_msgs__rosidl_typesupport_introspection_cpp.so"
  IMPORTED_SONAME_RELEASE "libobjectpose_msgs__rosidl_typesupport_introspection_cpp.so"
  )

list(APPEND _IMPORT_CHECK_TARGETS objectpose_msgs::objectpose_msgs__rosidl_typesupport_introspection_cpp )
list(APPEND _IMPORT_CHECK_FILES_FOR_objectpose_msgs::objectpose_msgs__rosidl_typesupport_introspection_cpp "${_IMPORT_PREFIX}/lib/libobjectpose_msgs__rosidl_typesupport_introspection_cpp.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
