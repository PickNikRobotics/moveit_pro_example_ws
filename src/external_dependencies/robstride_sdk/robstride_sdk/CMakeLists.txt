cmake_minimum_required(VERSION 3.10)
project(robstride_sdk VERSION 0.1.0 LANGUAGES CXX)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake QUIET)
include(CTest)

add_library(${PROJECT_NAME}
  SHARED
  src/can_bus.cpp
)

target_include_directories(${PROJECT_NAME}
  PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(${PROJECT_NAME} PUBLIC pthread)

install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

if(BUILD_TESTING)
  add_executable(test_fault_feedback test/test_fault_feedback.cpp)
  target_link_libraries(test_fault_feedback PRIVATE ${PROJECT_NAME})
  foreach(test_case routing routing_without_motor_zero fault_bits overtemperature
      warning_bits short_frame motor_feedback temperature_freshness)
    add_test(NAME fault_feedback_${test_case} COMMAND test_fault_feedback ${test_case})
  endforeach()
  if(ament_cmake_FOUND)
    find_package(ament_lint_auto REQUIRED)
    ament_lint_auto_find_test_dependencies()
  endif()
endif()

if(ament_cmake_FOUND)
  ament_export_include_directories(include)
  ament_export_libraries(${PROJECT_NAME})
  ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
  ament_package()
endif()
