# Modified by PickNik Inc., 2026.
file(REMOVE_RECURSE "${CONSUMER_BINARY_DIR}")

execute_process(
  COMMAND
    "${CMAKE_EXECUTABLE}" -S "${CONSUMER_SOURCE_DIR}" -B "${CONSUMER_BINARY_DIR}"
  RESULT_VARIABLE configure_result
)
if(NOT configure_result EQUAL 0)
  message(FATAL_ERROR "Installed-header consumer configuration failed: ${configure_result}")
endif()

execute_process(
  COMMAND "${CMAKE_EXECUTABLE}" --build "${CONSUMER_BINARY_DIR}"
  RESULT_VARIABLE build_result
)
if(NOT build_result EQUAL 0)
  message(FATAL_ERROR "Installed-header consumer build failed: ${build_result}")
endif()
