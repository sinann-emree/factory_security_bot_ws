cmake_minimum_required(VERSION 3.8)
project(fsb_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  # cmd_vel_watchdog: iletme, zaman aşımında sıfır hız, yeniden açılma
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_cmd_vel_watchdog test/test_cmd_vel_watchdog.py TIMEOUT 120)
endif()

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
  PATTERN ".gitkeep" EXCLUDE)

# Robot tarafı güvenlik düğümü: ros2 run fsb_control cmd_vel_watchdog.py
install(PROGRAMS
  scripts/cmd_vel_watchdog.py
  DESTINATION lib/${PROJECT_NAME})

ament_package()
