cmake_minimum_required(VERSION 3.22)
project(moveit_pro_sam3)

find_package(ament_cmake REQUIRED)

# The ONNX files are downloaded at configure time instead of being committed, so
# this repository does not redistribute Meta's SAM Materials. For an offline build,
# point SAM3_MODEL_URL at a file:// directory holding the four files.
set(SAM3_MODEL_URL "https://github.com/mamoll/sam3-onnx/releases/download/v1.0.0"
  CACHE STRING "Location of the SAM3 ONNX release assets")
set(SAM3_MODEL_DIR "${CMAKE_CURRENT_BINARY_DIR}/models")

set(SAM3_MODEL_FILES
  sam3_vision_encoder.onnx
  sam3_text_encoder.onnx
  sam3_geometry_encoder.onnx
  sam3_decoder.onnx)
set(SAM3_MODEL_SHA256S
  4e4de058c8566469b1aa39f81b67dcba8c9f4161cbd60b84b3d5655f452ffb2e
  a6cef9ee22aa9d165666b2753cc59a201fa90b6a399e1cd3f3e0a1e8ec38ac12
  fde841d45a0bf890d402236632e9dcaa6e5fd19e3c40fc48fe8cd1c6911f34eb
  4ae7cca96889f1c72063cdb5cb9115d240ce1c8e5d2d4dbd389bc73c7d3f68b9)

set(SAM3_MODEL_PATHS)
foreach(model_file model_sha256 IN ZIP_LISTS SAM3_MODEL_FILES SAM3_MODEL_SHA256S)
  set(model_path "${SAM3_MODEL_DIR}/${model_file}")
  list(APPEND SAM3_MODEL_PATHS "${model_path}")
  if(EXISTS "${model_path}")
    file(SHA256 "${model_path}" actual_sha256)
    if(actual_sha256 STREQUAL model_sha256)
      continue()
    endif()
  endif()

  set(model_url "${SAM3_MODEL_URL}/${model_file}")
  message(STATUS "Downloading SAM3 model file ${model_url}")
  # Without a timeout, a connection that stalls after connecting hangs configure silently.
  file(DOWNLOAD "${model_url}" "${model_path}.part"
    STATUS download_status TLS_VERIFY ON INACTIVITY_TIMEOUT 60)
  list(GET download_status 0 download_code)
  if(NOT download_code EQUAL 0)
    list(GET download_status 1 download_message)
    file(REMOVE "${model_path}.part")
    message(FATAL_ERROR
      "Could not download SAM3 model file ${model_url}: ${download_message}. "
      "Check network access to that URL, or set -DSAM3_MODEL_URL=file:///<dir> to a "
      "directory holding the files listed in ${CMAKE_CURRENT_SOURCE_DIR}/README.md. "
      "To leave SAM3 out of the build, run `moveit_pro configure --sam3-license decline`.")
  endif()
  file(SHA256 "${model_path}.part" actual_sha256)
  if(NOT actual_sha256 STREQUAL model_sha256)
    file(REMOVE "${model_path}.part")
    message(FATAL_ERROR
      "SAM3 model file ${model_url} has SHA-256 ${actual_sha256}, expected "
      "${model_sha256}, so it was not used. The upstream file differs from the one "
      "MoveIt Pro was tested with; contact PickNik support.")
  endif()
  file(RENAME "${model_path}.part" "${model_path}")
endforeach()

install(
  FILES
    ${SAM3_MODEL_PATHS}
  DESTINATION
    share/${PROJECT_NAME}/models
)

# License notices must remain available if the install tree is redistributed
# independently of the source workspace. Use an explicit copy because colcon's
# symlink install would otherwise make them symlinks back into the source tree.
install(
  CODE "
    file(MAKE_DIRECTORY \"\$ENV{DESTDIR}\${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}\")
    execute_process(
      COMMAND \"${CMAKE_COMMAND}\" -E copy_if_different
        \"${CMAKE_CURRENT_SOURCE_DIR}/LICENSE\"
        \"${CMAKE_CURRENT_SOURCE_DIR}/README.md\"
        \"\$ENV{DESTDIR}\${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}\"
      COMMAND_ERROR_IS_FATAL ANY
    )
    file(MAKE_DIRECTORY \"\$ENV{DESTDIR}\${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}/models\")
    execute_process(
      COMMAND \"${CMAKE_COMMAND}\" -E copy_if_different
        \"${CMAKE_CURRENT_SOURCE_DIR}/models/LICENSE\"
        \"\$ENV{DESTDIR}\${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}/models\"
      COMMAND_ERROR_IS_FATAL ANY
    )
  "
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # CONTRIBUTING.md is custom prose and matches none of ament_copyright's
  # bundled *_contributing.txt templates, so the check reports it as unknown.
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
