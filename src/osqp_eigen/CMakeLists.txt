cmake_minimum_required(VERSION 3.8)
project(osqp_eigen)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wno-error=uninitialized)
endif()

include(FetchContent)
FetchContent_Declare(
  mujoco
  GIT_REPOSITORY https://github.com/robotology/osqp-eigen.git
  GIT_TAG v0.9.0)

if(NOT mujoco_POPULATED)
  FetchContent_MakeAvailable(mujoco)
endif()
