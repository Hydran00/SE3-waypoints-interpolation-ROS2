cmake_minimum_required(VERSION 3.5)
project(interpolator)


if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


find_package(ament_cmake REQUIRED)

find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(pinocchio REQUIRED)
set(THIS_PACKAGE_INCLUDE_DEPENDS
  rclcpp
  tf2_ros
  geometry_msgs
  std_msgs
  pinocchio
)
#C++
add_executable(tool0_broadcaster src/tool0_broadcaster.cpp)
add_executable(interpolator src/interpolate.cpp)
ament_target_dependencies(tool0_broadcaster ${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_target_dependencies(interpolator ${THIS_PACKAGE_INCLUDE_DEPENDS})
install(TARGETS
tool0_broadcaster
interpolator
  DESTINATION lib/${PROJECT_NAME})
ament_package()
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# Install python scripts


