cmake_minimum_required(VERSION 3.5)
project(wit_node)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(wit_msgs REQUIRED)
find_package(wit_driver REQUIRED)

# executable
add_executable(wit_node
  src/wit_node.cpp
)
ament_target_dependencies(wit_node
  rclcpp
  std_msgs
  geometry_msgs
  sensor_msgs
  tf2
  wit_msgs
  wit_driver
)
target_include_directories(wit_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
install(TARGETS
wit_node
  DESTINATION lib/${PROJECT_NAME}
)

# component
add_library(wit_node_component SHARED
  src/wit_node.cpp
)
rclcpp_components_register_nodes(wit_node_component
 "wit::WitNode")
target_compile_definitions(wit_node_component
  PRIVATE "WIT_NODE_BUILDING_LIBRARY"
)
target_compile_options(wit_node_component PUBLIC -Wall)
target_include_directories(wit_node_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(wit_node_component
  rclcpp
  rclcpp_components
  std_msgs
  geometry_msgs
  sensor_msgs
  tf2
  wit_msgs
  wit_driver
  )
install(TARGETS wit_node_component
  EXPORT export_${PROJECT_NAME}
  DESTINATION lib
)

install(DIRECTORY
  # launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
