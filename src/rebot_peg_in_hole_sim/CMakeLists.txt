cmake_minimum_required(VERSION 3.22)
project(rebot_peg_in_hole_sim)
find_package(ament_cmake REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter)
find_package(rebot_bench_sim REQUIRED)
find_package(rebot_bench_base_config REQUIRED)
get_filename_component(BENCH_SHARE ${rebot_bench_sim_DIR}/.. ABSOLUTE)
get_filename_component(BASE_SHARE ${rebot_bench_base_config_DIR}/.. ABSOLUTE)
file(GLOB_RECURSE BASE_XACROS CONFIGURE_DEPENDS ${BASE_SHARE}/description/*.xacro)

set(GENERATED_DESCRIPTION ${CMAKE_CURRENT_BINARY_DIR}/description)
add_custom_command(
  OUTPUT ${GENERATED_DESCRIPTION}/scene.xml
         ${GENERATED_DESCRIPTION}/rebot_peg.urdf
         ${GENERATED_DESCRIPTION}/rebot_arm.xml
         ${GENERATED_DESCRIPTION}/rebot_assets.xml
         ${GENERATED_DESCRIPTION}/bench_layout.yaml
         ${GENERATED_DESCRIPTION}/mat_grid.png
  COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/scripts/assemble_scene.py
          ${CMAKE_CURRENT_SOURCE_DIR}/config/scene_snapshot.json
          ${GENERATED_DESCRIPTION}
          ${CMAKE_CURRENT_SOURCE_DIR}/description/assets
  DEPENDS scripts/assemble_scene.py config/scene_snapshot.json
          description/assets/part.stl description/assets/jig_base.stl
          description/assets/jig_post.stl description/assets/charuco_board.png
          description/assets/provenance.json
          ${BENCH_SHARE}/description/scene.xml
          ${BENCH_SHARE}/description/bench_layout.yaml
          ${BENCH_SHARE}/description/rebot_arm.xml
          ${BENCH_SHARE}/description/rebot_assets.xml
          ${BENCH_SHARE}/description/rebot_bench_sim.urdf.xacro ${BASE_XACROS}
  VERBATIM
)
add_custom_target(${PROJECT_NAME}_scene ALL
  DEPENDS ${GENERATED_DESCRIPTION}/scene.xml ${GENERATED_DESCRIPTION}/rebot_peg.urdf)
install(DIRECTORY ${GENERATED_DESCRIPTION}/ DESTINATION share/${PROJECT_NAME}/description)
install(DIRECTORY config objectives docs DESTINATION share/${PROJECT_NAME})
install(FILES README.md LICENSE DESTINATION share/${PROJECT_NAME})
if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_peg_scene test/test_scene.py TIMEOUT 120)
endif()
ament_package()
