# Install script for directory: /home/dwyackzan/moveit_pro_source/src/moveit_pro/moveit_studio_ws/moveit_pro_example_ws/src/example_behaviors

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/usr/local")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "Debug")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

# Set path to fallback-tool for dependency-resolution.
if(NOT DEFINED CMAKE_OBJDUMP)
  set(CMAKE_OBJDUMP "/usr/bin/objdump")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libexample_behaviors.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libexample_behaviors.so")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libexample_behaviors.so"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE SHARED_LIBRARY FILES "/home/dwyackzan/moveit_pro_source/src/moveit_pro/moveit_studio_ws/moveit_pro_example_ws/src/example_behaviors/cmake-build-debug/libexample_behaviors.so")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libexample_behaviors.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libexample_behaviors.so")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libexample_behaviors.so"
         OLD_RPATH "/home/dwyackzan/moveit_pro_source/install/moveit_studio_behavior/lib:/home/dwyackzan/moveit_pro_source/install/moveit_studio_behavior_interface/lib:/home/dwyackzan/moveit_pro_source/install/moveit_studio_vision/lib:/opt/ros/humble/lib/x86_64-linux-gnu:/home/dwyackzan/moveit_pro_source/install/moveit_msgs/lib:/home/dwyackzan/moveit_pro_source/install/moveit_core/lib:/opt/ros/humble/lib:/home/dwyackzan/moveit_pro_source/install/moveit_pro_ml/lib:/home/dwyackzan/moveit_pro_source/install/joint_trajectory_admittance_controller/lib:/home/dwyackzan/moveit_pro_source/install/ros2_control_utils/lib:/home/dwyackzan/moveit_pro_source/install/control_common/lib:/home/dwyackzan/moveit_pro_source/install/moveit_pro_controllers_msgs/lib:/home/dwyackzan/moveit_pro_source/install/pose_ik/lib:/home/dwyackzan/moveit_pro_source/install/moveit_pro_mpc/lib:/home/dwyackzan/moveit_pro_source/install/picknik_mujoco/lib:/home/dwyackzan/moveit_pro_source/install/moveit_pro_ml_ros/lib:/home/dwyackzan/moveit_pro_source/install/moveit_studio_agent_msgs/lib:/home/dwyackzan/moveit_pro_source/install/moveit_task_constructor_msgs/lib:/home/dwyackzan/moveit_pro_source/install/moveit_studio_sdk_msgs/lib:/home/dwyackzan/moveit_pro_source/install/moveit_studio_vision_msgs/lib:/home/dwyackzan/moveit_pro_source/install/moveit_studio_internal_msgs/lib:/home/dwyackzan/moveit_pro_source/install/pro_rrt/lib:/home/dwyackzan/moveit_pro_source/install/cartesian_planning/lib:/home/dwyackzan/moveit_pro_source/install/rrtconnect/lib:/home/dwyackzan/moveit_pro_source/install/moveit_studio_common/lib:/home/dwyackzan/moveit_pro_source/install/moveit_task_constructor_core/lib:/home/dwyackzan/moveit_pro_source/install/rviz_marker_tools/lib:/home/dwyackzan/moveit_pro_source/install/moveit_ros_planning_interface/lib:/home/dwyackzan/moveit_pro_source/install/moveit_ros_move_group/lib:/home/dwyackzan/moveit_pro_source/install/moveit_ros_warehouse/lib:/home/dwyackzan/moveit_pro_source/install/moveit_ros_planning/lib:/home/dwyackzan/moveit_pro_source/install/moveit_ros_occupancy_map_monitor/lib:/opt/ros/humble/opt/rviz_ogre_vendor/lib:/home/dwyackzan/moveit_pro_source/install/spdlog_ros/lib:/home/dwyackzan/moveit_pro_source/install/behaviortree_cpp/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libexample_behaviors.so")
    endif()
  endif()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/example_behaviors" TYPE DIRECTORY FILES "/home/dwyackzan/moveit_pro_source/src/moveit_pro/moveit_studio_ws/moveit_pro_example_ws/src/example_behaviors/config")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/home/dwyackzan/moveit_pro_source/src/moveit_pro/moveit_studio_ws/moveit_pro_example_ws/src/example_behaviors/cmake-build-debug/test/cmake_install.cmake")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/example_behaviors" TYPE FILE FILES "/home/dwyackzan/moveit_pro_source/src/moveit_pro/moveit_studio_ws/moveit_pro_example_ws/src/example_behaviors/example_behaviors_plugin_description.xml")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/example_behaviors/environment" TYPE FILE FILES "/opt/ros/humble/lib/python3.10/site-packages/ament_package/template/environment_hook/library_path.sh")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/example_behaviors/environment" TYPE FILE FILES "/home/dwyackzan/moveit_pro_source/src/moveit_pro/moveit_studio_ws/moveit_pro_example_ws/src/example_behaviors/cmake-build-debug/ament_cmake_environment_hooks/library_path.dsv")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ament_index/resource_index/package_run_dependencies" TYPE FILE FILES "/home/dwyackzan/moveit_pro_source/src/moveit_pro/moveit_studio_ws/moveit_pro_example_ws/src/example_behaviors/cmake-build-debug/ament_cmake_index/share/ament_index/resource_index/package_run_dependencies/example_behaviors")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ament_index/resource_index/parent_prefix_path" TYPE FILE FILES "/home/dwyackzan/moveit_pro_source/src/moveit_pro/moveit_studio_ws/moveit_pro_example_ws/src/example_behaviors/cmake-build-debug/ament_cmake_index/share/ament_index/resource_index/parent_prefix_path/example_behaviors")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/example_behaviors/environment" TYPE FILE FILES "/home/dwyackzan/moveit_pro_source/install/ament_cmake_core/share/ament_cmake_core/cmake/environment_hooks/environment/ament_prefix_path.sh")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/example_behaviors/environment" TYPE FILE FILES "/home/dwyackzan/moveit_pro_source/src/moveit_pro/moveit_studio_ws/moveit_pro_example_ws/src/example_behaviors/cmake-build-debug/ament_cmake_environment_hooks/ament_prefix_path.dsv")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/example_behaviors/environment" TYPE FILE FILES "/home/dwyackzan/moveit_pro_source/install/ament_cmake_core/share/ament_cmake_core/cmake/environment_hooks/environment/path.sh")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/example_behaviors/environment" TYPE FILE FILES "/home/dwyackzan/moveit_pro_source/src/moveit_pro/moveit_studio_ws/moveit_pro_example_ws/src/example_behaviors/cmake-build-debug/ament_cmake_environment_hooks/path.dsv")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/example_behaviors" TYPE FILE FILES "/home/dwyackzan/moveit_pro_source/src/moveit_pro/moveit_studio_ws/moveit_pro_example_ws/src/example_behaviors/cmake-build-debug/ament_cmake_environment_hooks/local_setup.bash")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/example_behaviors" TYPE FILE FILES "/home/dwyackzan/moveit_pro_source/src/moveit_pro/moveit_studio_ws/moveit_pro_example_ws/src/example_behaviors/cmake-build-debug/ament_cmake_environment_hooks/local_setup.sh")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/example_behaviors" TYPE FILE FILES "/home/dwyackzan/moveit_pro_source/src/moveit_pro/moveit_studio_ws/moveit_pro_example_ws/src/example_behaviors/cmake-build-debug/ament_cmake_environment_hooks/local_setup.zsh")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/example_behaviors" TYPE FILE FILES "/home/dwyackzan/moveit_pro_source/src/moveit_pro/moveit_studio_ws/moveit_pro_example_ws/src/example_behaviors/cmake-build-debug/ament_cmake_environment_hooks/local_setup.dsv")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/example_behaviors" TYPE FILE FILES "/home/dwyackzan/moveit_pro_source/src/moveit_pro/moveit_studio_ws/moveit_pro_example_ws/src/example_behaviors/cmake-build-debug/ament_cmake_environment_hooks/package.dsv")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ament_index/resource_index/packages" TYPE FILE FILES "/home/dwyackzan/moveit_pro_source/src/moveit_pro/moveit_studio_ws/moveit_pro_example_ws/src/example_behaviors/cmake-build-debug/ament_cmake_index/share/ament_index/resource_index/packages/example_behaviors")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ament_index/resource_index/moveit_studio_behavior_interface__pluginlib__plugin" TYPE FILE FILES "/home/dwyackzan/moveit_pro_source/src/moveit_pro/moveit_studio_ws/moveit_pro_example_ws/src/example_behaviors/cmake-build-debug/ament_cmake_index/share/ament_index/resource_index/moveit_studio_behavior_interface__pluginlib__plugin/example_behaviors")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/example_behaviors/cmake/example_behaviorsTargetsExport.cmake")
    file(DIFFERENT _cmake_export_file_changed FILES
         "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/example_behaviors/cmake/example_behaviorsTargetsExport.cmake"
         "/home/dwyackzan/moveit_pro_source/src/moveit_pro/moveit_studio_ws/moveit_pro_example_ws/src/example_behaviors/cmake-build-debug/CMakeFiles/Export/bf0e7080e44838dd029816bf02dac10d/example_behaviorsTargetsExport.cmake")
    if(_cmake_export_file_changed)
      file(GLOB _cmake_old_config_files "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/example_behaviors/cmake/example_behaviorsTargetsExport-*.cmake")
      if(_cmake_old_config_files)
        string(REPLACE ";" ", " _cmake_old_config_files_text "${_cmake_old_config_files}")
        message(STATUS "Old export file \"$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/example_behaviors/cmake/example_behaviorsTargetsExport.cmake\" will be replaced.  Removing files [${_cmake_old_config_files_text}].")
        unset(_cmake_old_config_files_text)
        file(REMOVE ${_cmake_old_config_files})
      endif()
      unset(_cmake_old_config_files)
    endif()
    unset(_cmake_export_file_changed)
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/example_behaviors/cmake" TYPE FILE FILES "/home/dwyackzan/moveit_pro_source/src/moveit_pro/moveit_studio_ws/moveit_pro_example_ws/src/example_behaviors/cmake-build-debug/CMakeFiles/Export/bf0e7080e44838dd029816bf02dac10d/example_behaviorsTargetsExport.cmake")
  if(CMAKE_INSTALL_CONFIG_NAME MATCHES "^([Dd][Ee][Bb][Uu][Gg])$")
    file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/example_behaviors/cmake" TYPE FILE FILES "/home/dwyackzan/moveit_pro_source/src/moveit_pro/moveit_studio_ws/moveit_pro_example_ws/src/example_behaviors/cmake-build-debug/CMakeFiles/Export/bf0e7080e44838dd029816bf02dac10d/example_behaviorsTargetsExport-debug.cmake")
  endif()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/example_behaviors/cmake" TYPE FILE FILES "/home/dwyackzan/moveit_pro_source/src/moveit_pro/moveit_studio_ws/moveit_pro_example_ws/src/example_behaviors/cmake-build-debug/ament_cmake_export_targets/ament_cmake_export_targets-extras.cmake")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/example_behaviors/cmake" TYPE FILE FILES "/home/dwyackzan/moveit_pro_source/src/moveit_pro/moveit_studio_ws/moveit_pro_example_ws/src/example_behaviors/cmake-build-debug/ament_cmake_export_dependencies/ament_cmake_export_dependencies-extras.cmake")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/example_behaviors/cmake" TYPE FILE FILES
    "/home/dwyackzan/moveit_pro_source/src/moveit_pro/moveit_studio_ws/moveit_pro_example_ws/src/example_behaviors/cmake-build-debug/ament_cmake_core/example_behaviorsConfig.cmake"
    "/home/dwyackzan/moveit_pro_source/src/moveit_pro/moveit_studio_ws/moveit_pro_example_ws/src/example_behaviors/cmake-build-debug/ament_cmake_core/example_behaviorsConfig-version.cmake"
    )
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/example_behaviors" TYPE FILE FILES "/home/dwyackzan/moveit_pro_source/src/moveit_pro/moveit_studio_ws/moveit_pro_example_ws/src/example_behaviors/package.xml")
endif()

string(REPLACE ";" "\n" CMAKE_INSTALL_MANIFEST_CONTENT
       "${CMAKE_INSTALL_MANIFEST_FILES}")
if(CMAKE_INSTALL_LOCAL_ONLY)
  file(WRITE "/home/dwyackzan/moveit_pro_source/src/moveit_pro/moveit_studio_ws/moveit_pro_example_ws/src/example_behaviors/cmake-build-debug/install_local_manifest.txt"
     "${CMAKE_INSTALL_MANIFEST_CONTENT}")
endif()
if(CMAKE_INSTALL_COMPONENT)
  if(CMAKE_INSTALL_COMPONENT MATCHES "^[a-zA-Z0-9_.+-]+$")
    set(CMAKE_INSTALL_MANIFEST "install_manifest_${CMAKE_INSTALL_COMPONENT}.txt")
  else()
    string(MD5 CMAKE_INST_COMP_HASH "${CMAKE_INSTALL_COMPONENT}")
    set(CMAKE_INSTALL_MANIFEST "install_manifest_${CMAKE_INST_COMP_HASH}.txt")
    unset(CMAKE_INST_COMP_HASH)
  endif()
else()
  set(CMAKE_INSTALL_MANIFEST "install_manifest.txt")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  file(WRITE "/home/dwyackzan/moveit_pro_source/src/moveit_pro/moveit_studio_ws/moveit_pro_example_ws/src/example_behaviors/cmake-build-debug/${CMAKE_INSTALL_MANIFEST}"
     "${CMAKE_INSTALL_MANIFEST_CONTENT}")
endif()
