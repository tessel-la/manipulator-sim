cmake_minimum_required(VERSION 3.22)
project(custom_servo_demo)

find_package(ament_cmake REQUIRED)

# Install launch, config directories
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(DIRECTORY worlds DESTINATION share/${PROJECT_NAME})
install(PROGRAMS
  scripts/robot_description_republisher.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME robot_description_republisher
)
install(PROGRAMS
  scripts/joy_to_servo_twist.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME joy_to_servo_twist
)
install(PROGRAMS
  scripts/servo_command_preparer.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME servo_command_preparer
)
install(PROGRAMS
  scripts/gazebo_wrist_camera_follower.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME gazebo_wrist_camera_follower
)
install(PROGRAMS
  scripts/pick_place_scene_publisher.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME pick_place_scene_publisher
)

ament_package()
