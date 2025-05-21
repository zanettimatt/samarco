cmake_minimum_required(VERSION 3.8)
project(magnetic_adhesion_plugin)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Add compiler warnings
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)  # Add std_msgs dependency
find_package(ignition-gazebo6 REQUIRED)
find_package(ignition-cmake2 REQUIRED)
find_package(ignition-plugin1 REQUIRED COMPONENTS register)

# Include directories
include_directories(${ignition-gazebo6_INCLUDE_DIRS})

# Add library
add_library(magnetic_adhesion_plugin src/MagneticAdhesionPlugin.cpp)

# Set compile features
target_compile_features(magnetic_adhesion_plugin PUBLIC cxx_std_17)

# Include directories for the library
target_include_directories(magnetic_adhesion_plugin PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Add dependencies
ament_target_dependencies(
  magnetic_adhesion_plugin
  "rclcpp"
  "sensor_msgs"
  "std_msgs"  # Add std_msgs dependency
  "ignition-gazebo6"
)

# Link libraries
target_link_libraries(magnetic_adhesion_plugin
  ${ignition-gazebo6_LIBRARIES}
)

# Add compile definitions
target_compile_definitions(magnetic_adhesion_plugin PRIVATE "MAGNETIC_ADHESION_PLUGIN_BUILDING_LIBRARY")

# Install include directory
install(
  DIRECTORY include/
  DESTINATION include
)

# Install config directory
install(
  DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/
)

# Install the library
install(
  TARGETS magnetic_adhesion_plugin
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Enable testing (optional)
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # Skip copyright and cpplint checks (remove these lines if you want to enforce them)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# Export include directories, libraries, and targets
ament_export_include_directories(include)
ament_export_libraries(magnetic_adhesion_plugin)
ament_export_targets(export_${PROJECT_NAME})

# Finalize the package
ament_package()
