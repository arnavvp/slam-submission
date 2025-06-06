cmake_minimum_required(VERSION 3.8)
project(measurement_update)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(eufs_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(dv_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

add_executable(ekf ekf.cpp data_assoc_trainee.cpp)
add_executable(ekf_marker ekf_marker.cpp)
add_executable(NN_data_assoc NN_data_assoc.cpp)


target_include_directories(ekf_marker PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_include_directories(ekf PUBLIC
 $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIR}
)

arget_include_directories(NN_data_assoc PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIR}
)

ament_target_dependencies(ekf
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  visualization_msgs
  eufs_msgs
  dv_msgs
)
ament_target_dependencies(ekf_marker
  rclcpp
  std_msgs
  sensor_msgs
  visualization_msgs
  eufs_msgs
  dv_msgs
)

ament_target_dependencies(ekf_marker rclcpp std_msgs sensor_msgs visualization_msgs eufs_msgs dv_msgs)

ament_target_dependencies(NN_data_assoc
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  visualization_msgs
  eufs_msgs
  dv_msgs
)

target_link_libraries(NN_data_assoc Eigen3::Eigen)

install(TARGETS
  ekf_marker
  ekf
  NN_data_assoc
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
