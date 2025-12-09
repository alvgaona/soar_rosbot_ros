cmake_minimum_required(VERSION 3.8)
project(soar_rosbot_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 20)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(soar_ros REQUIRED)
find_package(ament_index_cpp REQUIRED)

set(THIS_PACKAGE_INCLUDE_DEPENDS
  rclcpp
  std_msgs
  geometry_msgs
  soar_ros
  ament_index_cpp
)

# Include directories
include_directories(
  include
)

# Create library
add_library(${PROJECT_NAME}_lib
  src/soar_maze_controller.cpp
  src/motion_controller.cpp
)

ament_target_dependencies(${PROJECT_NAME}_lib
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

# Create Soar maze controller executable
add_executable(soar_maze_controller
  src/main.cpp
)

target_link_libraries(soar_maze_controller
  ${PROJECT_NAME}_lib
)

ament_target_dependencies(soar_maze_controller
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

# Create motion controller executable
add_executable(motion_controller
  src/motion_controller_main.cpp
)

target_link_libraries(motion_controller
  ${PROJECT_NAME}_lib
)

ament_target_dependencies(motion_controller
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

# Install targets
install(TARGETS
  soar_maze_controller
  motion_controller
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  ${PROJECT_NAME}_lib
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install header files
install(DIRECTORY include/
  DESTINATION include
)

# Install Soar rules directory
install(DIRECTORY soar_rules/
  DESTINATION share/${PROJECT_NAME}/soar_rules
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME}_lib)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})

ament_package()
