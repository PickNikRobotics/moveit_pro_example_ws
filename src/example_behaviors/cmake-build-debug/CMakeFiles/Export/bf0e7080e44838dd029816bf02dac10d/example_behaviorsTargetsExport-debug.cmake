#----------------------------------------------------------------
# Generated CMake target import file for configuration "Debug".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "example_behaviors::example_behaviors" for configuration "Debug"
set_property(TARGET example_behaviors::example_behaviors APPEND PROPERTY IMPORTED_CONFIGURATIONS DEBUG)
set_target_properties(example_behaviors::example_behaviors PROPERTIES
  IMPORTED_LOCATION_DEBUG "${_IMPORT_PREFIX}/lib/libexample_behaviors.so"
  IMPORTED_SONAME_DEBUG "libexample_behaviors.so"
  )

list(APPEND _cmake_import_check_targets example_behaviors::example_behaviors )
list(APPEND _cmake_import_check_files_for_example_behaviors::example_behaviors "${_IMPORT_PREFIX}/lib/libexample_behaviors.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
