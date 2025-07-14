cmake_minimum_required(VERSION 3.8)
project(elevation_map_node)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(PCL REQUIRED)
find_package(pcl_conversions REQUIRED)

# Add executables
add_executable(elevation_map_generator src/elevation_map_generator.cpp)

# Include directories
target_include_directories(elevation_map_generator PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${PCL_INCLUDE_DIRS})

# Target compile features
target_compile_features(elevation_map_generator PUBLIC c_std_99 cxx_std_17)

# Dependencies
ament_target_dependencies(elevation_map_generator
  rclcpp
  sensor_msgs
  nav_msgs
  geometry_msgs
  std_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  pcl_conversions)

# Link PCL libraries
target_link_libraries(elevation_map_generator ${PCL_LIBRARIES})

# Add compile definitions
target_compile_definitions(elevation_map_generator PRIVATE ${PCL_DEFINITIONS})

# Install targets
install(TARGETS 
  elevation_map_generator
  DESTINATION lib/${PROJECT_NAME})

# Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

# Install config files
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package() 