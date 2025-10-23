cmake_minimum_required(VERSION 3.22)
project(crx_hw)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY
    config
    launch
    objectives
  DESTINATION
    share/${PROJECT_NAME}
)

# Install the Python script as a “binary” in lib/<pkg>
install(PROGRAMS scripts/gripper_gpio_service.py
    DESTINATION lib/${PROJECT_NAME}
    RENAME gripper_gpio_service)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
