cmake_minimum_required(VERSION 3.22)
project(behavior_hub_catalog)

find_package(ament_cmake REQUIRED)

# Config-only package: it inherits everything else from lab_sim via
# `based_on_package`, so only the config directory is installed.
install(
  DIRECTORY
    config
  DESTINATION
    share/${PROJECT_NAME}
)

ament_package()
