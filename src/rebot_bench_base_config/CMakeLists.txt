cmake_minimum_required(VERSION 3.22)
project(rebot_bench_base_config)
find_package(ament_cmake REQUIRED)
install(DIRECTORY config description docs launch objectives waypoints DESTINATION share/${PROJECT_NAME})
install(FILES README.md LICENSE thumbnail.png DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/scale_depth.py test/verify_mock_execution.py test/verify_mock_gripper_jog.py
  scripts/rebot_leader.py scripts/rebot_leader_bridge.py scripts/rebot_leader_tune.py
  DESTINATION lib/${PROJECT_NAME})
install(FILES scripts/rebot_leader_state.py scripts/rebot_leader_collision.py
  DESTINATION lib/${PROJECT_NAME})
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(behaviortree_cpp_picknik REQUIRED)
  find_package(tinyxml2_vendor REQUIRED)
  find_package(TinyXML2 REQUIRED)
  find_package(yaml-cpp REQUIRED)
  find_package(moveit_pro_base REQUIRED)
  find_package(urdf REQUIRED)
  find_package(srdfdom REQUIRED)
  find_package(rebot_bench_behaviors REQUIRED)
  ament_add_gtest(test_stack_objectives test/test_stack_objectives.cpp)
  ament_target_dependencies(test_stack_objectives behaviortree_cpp_picknik)
  target_compile_features(test_stack_objectives PRIVATE cxx_std_20)
  # Header-only motion settings parser; the Behavior library itself is not linked.
  target_include_directories(test_stack_objectives PRIVATE
    $<TARGET_PROPERTY:rebot_bench_behaviors::rebot_bench_behaviors,INTERFACE_INCLUDE_DIRECTORIES>)
  target_link_libraries(test_stack_objectives tinyxml2::tinyxml2 yaml-cpp::yaml-cpp)
  target_compile_definitions(test_stack_objectives PRIVATE PACKAGE_DIR="${CMAKE_CURRENT_SOURCE_DIR}")
  set(TEST_URDF ${CMAKE_CURRENT_BINARY_DIR}/test_rebot.urdf)
  add_custom_command(
    OUTPUT ${TEST_URDF}
    COMMAND xacro ${CMAKE_CURRENT_SOURCE_DIR}/description/rebot.urdf.xacro
            hardware_interface:=mock -o ${TEST_URDF}
    DEPENDS description/rebot.urdf.xacro
    VERBATIM)
  add_custom_target(test_rebot_urdf DEPENDS ${TEST_URDF})
  ament_add_gtest(test_joint_limits test/test_joint_limits.cpp)
  add_dependencies(test_joint_limits test_rebot_urdf)
  ament_target_dependencies(test_joint_limits moveit_pro_base urdf srdfdom)
  target_link_libraries(test_joint_limits yaml-cpp::yaml-cpp)
  target_compile_definitions(test_joint_limits PRIVATE
    TEST_URDF="${TEST_URDF}" PACKAGE_DIR="${CMAKE_CURRENT_SOURCE_DIR}")
  # The jaw, waypoint and leader HOME legs with the real reBot Behaviors and robot model.
  ament_add_gtest(test_jaw_legs test/test_jaw_legs.cpp)
  add_dependencies(test_jaw_legs test_rebot_urdf)
  target_compile_features(test_jaw_legs PRIVATE cxx_std_20)
  ament_target_dependencies(test_jaw_legs moveit_pro_base urdf srdfdom)
  target_link_libraries(test_jaw_legs rebot_bench_behaviors::rebot_bench_behaviors)
  target_compile_definitions(test_jaw_legs PRIVATE
    TEST_URDF="${TEST_URDF}" PACKAGE_DIR="${CMAKE_CURRENT_SOURCE_DIR}")
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_stack_legs test/test_stack_legs.py TIMEOUT 60)
  ament_add_pytest_test(test_scale_depth test/test_scale_depth.py TIMEOUT 60)
  ament_add_pytest_test(test_hardware_selection test/test_hardware_selection.py TIMEOUT 60)
  ament_add_pytest_test(test_rebot_leader test/test_rebot_leader.py TIMEOUT 60)
  ament_add_pytest_test(test_rebot_model test/test_model.py TIMEOUT 60)
endif()
ament_package()
