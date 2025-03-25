cmake_minimum_required(VERSION 3.8)
project(optical_flow)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_runtime REQUIRED)
find_package(Boost REQUIRED)
find_package(optical_flow_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)

include_directories(
  include
  ${Boost_INCLUDE_DIRS}
  ${rclcpp_INCLUDE_DIRS}
  ${optical_flow_msgs_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

add_executable(optical_flow_node src/optical_flow_node.cpp)
target_include_directories(optical_flow_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_link_libraries(optical_flow_node PRIVATE ${Boost_LIBRARIES})
ament_target_dependencies(optical_flow_node rclcpp optical_flow_msgs)
target_compile_features(optical_flow_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

add_executable(flow_aggregator_node src/flow_aggregator_node.cpp)
target_include_directories(flow_aggregator_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
ament_target_dependencies(flow_aggregator_node rclcpp optical_flow_msgs geometry_msgs)
target_compile_features(flow_aggregator_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

install(TARGETS optical_flow_node flow_aggregator_node
  DESTINATION lib/${PROJECT_NAME})

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
