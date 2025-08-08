cmake_minimum_required(VERSION 3.8)
project(arm_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# macOS用のrpath設定
# if(APPLE)
#   set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)
#   set(CMAKE_BUILD_RPATH_USE_ORIGIN TRUE)
#   set(CMAKE_INSTALL_RPATH "@loader_path/../lib")
# endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)

# include directories
include_directories(include)

# create library
add_library(${PROJECT_NAME}_lib
  src/arm.cpp
  src/core.cpp
  src/helper.cpp
)

ament_target_dependencies(${PROJECT_NAME}_lib
  rclcpp
  geometry_msgs
  sensor_msgs
  std_msgs
  visualization_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  Eigen3
)

# ライブラリのプロパティを設定
set_target_properties(${PROJECT_NAME}_lib PROPERTIES
  POSITION_INDEPENDENT_CODE ON
)

# create test executable
add_executable(test_node test/test.cpp)
target_link_libraries(test_node ${PROJECT_NAME}_lib)

ament_target_dependencies(test_node
  rclcpp
  geometry_msgs
  sensor_msgs
  std_msgs
  visualization_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  Eigen3
)

# install
install(TARGETS
  test_node
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  ${PROJECT_NAME}_lib
  EXPORT ${PROJECT_NAME}_libTargets
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY
  include/
  DESTINATION include
)

install(DIRECTORY
  launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

install(DIRECTORY
  rviz/
  DESTINATION share/${PROJECT_NAME}/rviz
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()