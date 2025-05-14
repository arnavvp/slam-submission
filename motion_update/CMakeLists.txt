cmake_minimum_required(VERSION 3.8)
project(motion_update)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(eufs_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
#find_package(dv_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

add_executable(motion_update_node motion_update_node.cpp)
add_executable(car_marker car_marker.cpp)
#add_executable(ekf ekf.cpp data_assoc.cpp)



target_include_directories(motion_update_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

#target_include_directories(ekf PUBLIC
 # $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
 # $<INSTALL_INTERFACE:include>
 # ${EIGEN3_INCLUDE_DIR}
#)


ament_target_dependencies(motion_update_node
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  visualization_msgs
  eufs_msgs
  #dv_msgs
)

ament_target_dependencies(car_marker rclcpp std_msgs sensor_msgs visualization_msgs eufs_msgs)

#ament_target_dependencies(ekf
  #rclcpp
  #std_msgs
  #sensor_msgs
  #geometry_msgs
  #visualization_msgs
  #eufs_msgs
  #dv_msgs
#)



#target_link_libraries(ekf Eigen3::Eigen)

install(TARGETS
  motion_update_node
  car_marker
  #ekf
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
