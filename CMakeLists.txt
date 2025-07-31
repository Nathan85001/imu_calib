cmake_minimum_required(VERSION 3.8)
project(imu_calib)

# Use modern C++
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Compiler flags
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)  
find_package(ament_cmake_python REQUIRED)

# Set common dependencies in a variable
set(imu_calib_deps
  rclcpp
  sensor_msgs
  std_msgs
  yaml-cpp
  Eigen3
)

# Include directories (global)
#include_directories(include)

# === Build shared library from core components ===
add_library(imu_calib_lib
  src/apply_calib.cpp
  src/accel_calib/accel_calib.cpp
  src/do_calib.cpp  
)

target_include_directories(imu_calib_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(imu_calib_lib yaml-cpp)
ament_target_dependencies(imu_calib_lib ${imu_calib_deps})

# === Build executables ===

# apply_calib_node
add_executable(apply_calib_node src/apply_calib_node.cpp)
target_link_libraries(apply_calib_node imu_calib_lib)
ament_target_dependencies(apply_calib_node ${imu_calib_deps})

# do_calib_node
add_executable(do_calib_node src/do_calib_node.cpp)
target_link_libraries(do_calib_node imu_calib_lib)
ament_target_dependencies(do_calib_node ${imu_calib_deps})



# === Install binaries ===
install(TARGETS
  imu_calib_lib
  apply_calib_node
  do_calib_node


  DESTINATION lib/${PROJECT_NAME}
)

# === Install headers ===
install(DIRECTORY include/
  DESTINATION include/
)


# === Linting and testing ===
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# Export dependencies used in public headers
ament_export_dependencies(eigen3_cmake_module)
ament_export_dependencies(Eigen3)

ament_package()
