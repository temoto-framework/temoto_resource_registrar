#----------------------------------------------------------------
# Generated CMake target import file.
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "temoto_resource_registrar" for configuration ""
set_property(TARGET temoto_resource_registrar APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(temoto_resource_registrar PROPERTIES
  IMPORTED_LOCATION_NOCONFIG "${_IMPORT_PREFIX}/lib/libtemoto_resource_registrar.so.0.0.1"
  IMPORTED_SONAME_NOCONFIG "libtemoto_resource_registrar.so.0.0.1"
  )

list(APPEND _IMPORT_CHECK_TARGETS temoto_resource_registrar )
list(APPEND _IMPORT_CHECK_FILES_FOR_temoto_resource_registrar "${_IMPORT_PREFIX}/lib/libtemoto_resource_registrar.so.0.0.1" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
