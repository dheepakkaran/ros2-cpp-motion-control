cmake_minimum_required(VERSION 3.16)
project(arm_motion_control LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(mujoco REQUIRED)

set(DEFAULT_MODEL "${CMAKE_CURRENT_SOURCE_DIR}/assets/franka_emika_panda/panda_nohand.xml")

# Header-only trajectory / watchdog / controller library (no ROS).
add_library(amc_core INTERFACE)
target_include_directories(amc_core INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> $<INSTALL_INTERFACE:include>)

# MuJoCo C API sim wrapper.
add_library(amc_sim src/mujoco_sim.cpp)
target_link_libraries(amc_sim PUBLIC amc_core mujoco::mujoco)

# ROS 2 node library + executable.
add_library(amc_node src/motion_control_node.cpp)
target_link_libraries(amc_node PUBLIC amc_sim rclcpp::rclcpp
  ${sensor_msgs_TARGETS} ${trajectory_msgs_TARGETS})
target_compile_definitions(amc_node PUBLIC ARM_MOTION_CONTROL_DEFAULT_MODEL="${DEFAULT_MODEL}")

add_executable(motion_control_node src/main.cpp)
target_link_libraries(motion_control_node amc_node)

# Demo: logs every 1 ms tick of two fault-injection scenarios to CSV (no ROS needed).
add_executable(trajectory_demo tools/trajectory_demo.cpp)
target_link_libraries(trajectory_demo amc_sim)

install(TARGETS motion_control_node trajectory_demo amc_node amc_sim DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  set(RESULTS_DIR "${CMAKE_CURRENT_SOURCE_DIR}/results")
  ament_add_gtest(test_trajectory test/test_trajectory.cpp)
  target_link_libraries(test_trajectory amc_core)
  ament_add_gtest(test_watchdog test/test_watchdog.cpp)
  target_link_libraries(test_watchdog amc_core)
  ament_add_gtest(test_fault_injection test/test_fault_injection.cpp TIMEOUT 120 SKIP_LINKING_MAIN_LIBRARIES)
  target_link_libraries(test_fault_injection amc_sim)
  target_compile_definitions(test_fault_injection PRIVATE
    MODEL_PATH="${DEFAULT_MODEL}" RESULTS_DIR="${RESULTS_DIR}")
  ament_add_gtest(test_integration test/test_integration.cpp TIMEOUT 120 SKIP_LINKING_MAIN_LIBRARIES)
  target_link_libraries(test_integration amc_node)
  target_compile_definitions(test_integration PRIVATE RESULTS_DIR="${RESULTS_DIR}")
endif()

ament_package()
