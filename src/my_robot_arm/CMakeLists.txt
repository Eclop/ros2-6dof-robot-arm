cmake_minimum_required(VERSION 3.8)
project(my_robot_arm)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(xacro REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# ---------------------------------------------------------
# 1. INSTALACIÓN DE RECURSOS (Launch, URDF, Meshes)
# ---------------------------------------------------------
install(DIRECTORY
  launch
  meshes
  urdf
  DESTINATION share/${PROJECT_NAME}
)

# ---------------------------------------------------------
# 2. INSTALACIÓN DE SCRIPTS PYTHON (NUEVO)
# Esto permite ejecutarlos con: ros2 run my_robot_arm script.py
# ---------------------------------------------------------
install(PROGRAMS
  scripts/simple_move.py
  scripts/simple_pick.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()