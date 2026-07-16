cmake_minimum_required(VERSION 3.22)
project(licensing_example CXX)

find_package(moveit_pro_package REQUIRED)
moveit_pro_package()

find_package(moveit_studio_licensing REQUIRED)

# studio_licensing is a static library, so its private link dependencies are
# propagated to consumers at final link time. Only tl_expected and OpenSSL are
# re-exported by moveit_studio_licensing; the rest must be found here so their
# IMPORTED targets exist when we link.
find_package(moveit_pro_macros REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(fmt REQUIRED)
# tinyxml2_vendor supplies the FindTinyXML2 module; find it first so the module
# is on CMAKE_MODULE_PATH before resolving the TinyXML2 target.
find_package(tinyxml2_vendor REQUIRED)
find_package(TinyXML2 REQUIRED)

# A user-workspace binary links the exported licensing target. The header is
# pulled in transitively via the target's INTERFACE include directories.
add_executable(check_license src/check_license.cpp)
target_link_libraries(check_license
                      PRIVATE moveit_studio_licensing::studio_licensing)

install(TARGETS check_license DESTINATION lib/${PROJECT_NAME})

ament_package()
