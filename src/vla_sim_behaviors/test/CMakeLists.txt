find_package(ament_cmake_gtest REQUIRED)
find_package(ament_cmake_gmock REQUIRED)

ament_add_gtest(test_behavior_plugins test_behavior_plugins.cpp)
ament_target_dependencies(test_behavior_plugins ${THIS_PACKAGE_INCLUDE_DEPENDS})

ament_add_gmock(test_cube_stack_trial test_cube_stack_trial.cpp)
target_link_libraries(test_cube_stack_trial vla_sim_behaviors)

ament_add_gmock(test_sample_cube_stack_trial test_sample_cube_stack_trial.cpp)
target_link_libraries(test_sample_cube_stack_trial vla_sim_behaviors)
ament_target_dependencies(test_sample_cube_stack_trial
                          ${THIS_PACKAGE_INCLUDE_DEPENDS})
