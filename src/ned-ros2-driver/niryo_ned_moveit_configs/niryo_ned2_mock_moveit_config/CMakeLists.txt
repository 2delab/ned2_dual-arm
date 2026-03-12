cmake_minimum_required(VERSION 3.22)
project(niryo_ned2_mock_moveit_config)

find_package(ament_cmake REQUIRED)

ament_package()

# Install Python scripts
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/src")
  install(
    PROGRAMS src/load_workspace_scene.py
    DESTINATION lib/${PROJECT_NAME})
endif()

# Install launch files
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/launch")
  install(
    DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}
    PATTERN "setup_assistant.launch" EXCLUDE)
endif()

# Install config files
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/config")
  install(
    DIRECTORY config
    DESTINATION share/${PROJECT_NAME})
endif()

# Install setup assistant marker
install(
  FILES .setup_assistant
  DESTINATION share/${PROJECT_NAME})
