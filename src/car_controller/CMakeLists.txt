cmake_minimum_required(VERSION 3.8)
project(car_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(car_control_msgs REQUIRED)

include_directories(include)

set(dependencies
  rclcpp
  geometry_msgs
  nav_msgs
  sensor_msgs
  std_msgs
  tf2
  tf2_ros
  car_control_msgs
)

# --- Longitudinal controllers (new Autoware-style) ---
add_executable(fuzzy_pid_controller src/fuzzy_pid_controller.cpp)
ament_target_dependencies(fuzzy_pid_controller ${dependencies})

add_executable(pid_velocity_controller src/pid_velocity_controller.cpp)
ament_target_dependencies(pid_velocity_controller ${dependencies})

# --- Longitudinal vehicle simulator (new plant) ---
add_executable(longitudinal_vehicle_simulator src/longitudinal_vehicle_simulator.cpp)
ament_target_dependencies(longitudinal_vehicle_simulator ${dependencies})

# --- Legacy / utility nodes (kept for smoke test / diagnostics) ---
add_executable(simple_controller src/simple_controller.cpp)
ament_target_dependencies(simple_controller ${dependencies})
target_link_libraries(simple_controller Eigen3::Eigen)

add_executable(noisy_controller src/noisy_controller.cpp)
ament_target_dependencies(noisy_controller ${dependencies})

add_executable(velocity_logger src/velocity_logger.cpp)
ament_target_dependencies(velocity_logger ${dependencies})

foreach(target
  fuzzy_pid_controller
  pid_velocity_controller
  longitudinal_vehicle_simulator
  simple_controller
  noisy_controller
  velocity_logger
)
  target_include_directories(${target} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
  target_compile_features(${target} PUBLIC cxx_std_17)
endforeach()

install(
  DIRECTORY include
  DESTINATION include
)
install(TARGETS
  fuzzy_pid_controller
  pid_velocity_controller
  longitudinal_vehicle_simulator
  simple_controller
  noisy_controller
  velocity_logger
  DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY config launch worlds
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  # --- Unit tests ---
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_pid_core test/test_pid_core.cpp)
  target_include_directories(test_pid_core PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)

  ament_add_gtest(test_fuzzy_pid_core test/test_fuzzy_pid_core.cpp)
  target_include_directories(test_fuzzy_pid_core PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)

  ament_add_gtest(test_longitudinal_vehicle_model test/test_longitudinal_vehicle_model.cpp)
  target_include_directories(test_longitudinal_vehicle_model PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)

  ament_add_gtest(test_seeded_noise test/test_seeded_noise.cpp)
  target_include_directories(test_seeded_noise PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)

  # --- Integration tests ---
  find_package(launch_testing_ament_cmake REQUIRED)
  add_launch_test(test/test_longitudinal_pipeline.test.py)
endif()

ament_export_include_directories(include)
ament_export_dependencies(${dependencies})
ament_package()
