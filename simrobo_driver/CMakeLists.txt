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

set(driver_executable driver)
set(driver_library ${driver_executable}_core)
set(dependencies
  rclcpp
  rclcpp_components
  std_msgs
  geometry_msgs
  nav_msgs
  tf2
  tf2_ros
)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

include_directories(
  include
)

add_executable(${driver_executable}
  src/driver_node.cpp
  src/driver_component.cpp
)
add_library(${driver_library} SHARED
  src/driver_component.cpp
)
ament_target_dependencies(${driver_executable}
  ${dependencies}
)
ament_target_dependencies(${driver_library}
  ${dependencies}
)
rclcpp_components_register_nodes(${driver_library}
  "simrobo_driver::Driver"
)

install(DIRECTORY include/
  DESTINATION include/
)
install(TARGETS  ${driver_executable}
  DESTINATION lib/${PROJECT_NAME}
)
install(TARGETS  ${driver_library}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/
  DESTINATION include/
)
install(DIRECTORY launch cfg
  DESTINATION share/${PROJECT_NAME}
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

ament_export_include_directories(include)
ament_export_libraries(
  ${driver_executable}
  ${driver_library}
)
ament_export_dependencies(${dependencies})
ament_package()
