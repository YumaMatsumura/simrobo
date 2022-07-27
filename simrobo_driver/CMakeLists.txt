cmake_minimum_required(VERSION 3.5)
project(simrobo_driver)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
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
find_package(nav_msgs REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(tf2 REQUIRED)

include_directories(
  include
)
add_library(driver_library SHARED
  src/driver_component.cpp
)
ament_target_dependencies(driver_library
  rclcpp
  rclcpp_components
  std_msgs
  geometry_msgs
  nav_msgs
  tf2_msgs
  tf2
)
rclcpp_components_register_nodes(driver_library
  "simrobo_driver::Driver"
)

install(TARGETS
  driver_library
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib
)
install(DIRECTORY include/
  DESTINATION include/
)
install(DIRECTORY launch cfg
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(driver_library)
ament_export_dependencies(
  rclcpp
  rclcpp_components
  std_msgs
  geometry_msgs
  nav_msgs
  tf2_msgs
  tf2
)
ament_package()
