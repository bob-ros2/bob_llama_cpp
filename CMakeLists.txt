cmake_minimum_required(VERSION 3.8)
project(bob_llama_cpp)

find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)

# Install launch and config files.
install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}/
)

# Install Python executables
ament_python_install_package(${PROJECT_NAME}
  SETUP_CFG
    ${PROJECT_NAME}/setup.cfg
  SCRIPTS_DESTINATION
    lib/${PROJECT_NAME}
)

ament_package()
