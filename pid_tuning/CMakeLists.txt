cmake_minimum_required(VERSION 3.8)
project(pid_tuning)

# Infraestructura del paquete ROS 2
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# Generación del mensaje EvolutiveInfo
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/EvolutiveInfo.msg"
  DEPENDENCIES std_msgs
)

# Obtener la ubicación de instalación de paquetes Python
ament_get_python_install_dir(PYTHON_INSTALL_DIR)

# Instalar la biblioteca Python conservando la estructura original:
# src/pid_tuning/evolutive_algorithms
# src/pid_tuning/settings
install(
  DIRECTORY src/pid_tuning/
  DESTINATION ${PYTHON_INSTALL_DIR}/pid_tuning
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.pyc" EXCLUDE
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
