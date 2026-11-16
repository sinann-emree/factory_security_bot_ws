cmake_minimum_required(VERSION 3.8)
project(fsb_gazebo)

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
endif()

install(DIRECTORY worlds models config launch
  DESTINATION share/${PROJECT_NAME}
  PATTERN ".gitkeep" EXCLUDE)

# Test araçları: ros2 run fsb_gazebo <script>
install(PROGRAMS
  scripts/drive_test.py
  scripts/teleop_test.py
  scripts/scan_alignment_check.py
  scripts/mapping_tour.py
  scripts/map_quality_check.py
  scripts/amcl_check.py
  scripts/localization_monitor.py
  scripts/dynamic_obstacle_test.py
  scripts/recovery_counter.py
  scripts/gt_pose_publisher.py
  scripts/slam_replay_eval.py
  scripts/generate_factory_world.py
  scripts/clearance_monitor.py
  scripts/perf_monitor.py
  scripts/alert_response_monitor.py
  scripts/posture_monitor.py
  DESTINATION lib/${PROJECT_NAME})

ament_package()
