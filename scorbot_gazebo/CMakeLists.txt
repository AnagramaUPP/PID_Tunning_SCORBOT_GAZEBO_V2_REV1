cmake_minimum_required(VERSION 3.8)
project(scorbot_gazebo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(gz-sim8 REQUIRED)
find_package(gz-plugin2 REQUIRED COMPONENTS register)
find_package(gz-transport13 REQUIRED)
find_package(gz-msgs10 REQUIRED)

# Plugin de Gazebo para restablecer directamente el estado
# articular del Scorbot sin eliminar el modelo ni reiniciar el mundo.
add_library(
  scorbot_home_state_system
  SHARED
  src/scorbot_home_state_system.cpp
)

target_compile_features(
  scorbot_home_state_system
  PUBLIC
  cxx_std_17
)

target_link_libraries(
  scorbot_home_state_system
  PRIVATE
  gz-sim8::gz-sim8
  gz-plugin2::gz-plugin2
  gz-transport13::gz-transport13
  gz-msgs10::gz-msgs10
)

install(
  TARGETS
    scorbot_home_state_system
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY
    launch
    urdf
    meshes
    config
    trajectories
    worlds
    rviz
    scripts
  DESTINATION
    share/${PROJECT_NAME}
)
install(
  PROGRAMS
    scripts/cartesian_tracking_plotter.py
    scripts/cartesian_tracking_logger.py
    scripts/scorbot_trajectory_publisher.py
    scripts/scorbot_ik_publisher.py
    scripts/scorbot_joint_error_monitor.py
    scripts/tuning_node_de.py
    scripts/scorbot_pid_reset_supervisor.py
    scripts/run_test_pid.sh
    scripts/run_tuning_de.sh
    scripts/test_fixed_pid.py
    scripts/generate_lemniscata_trajectory.py
    scripts/run_30.sh
  DESTINATION
    lib/${PROJECT_NAME}
)
ament_package()