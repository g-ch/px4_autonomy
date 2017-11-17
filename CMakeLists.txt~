cmake_minimum_required(VERSION 2.8.3)
project(px4_autonomy)


find_package(catkin REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  roscpp
  tf
)

add_message_files(FILES
  Velocity.msg
  Position.msg
  Takeoff.msg
)


generate_messages(DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS
  message_runtime
)

include_directories(
# include
${catkin_INCLUDE_DIRS}
)

add_executable(offboard_center
  src/offboard_center.cpp
)

target_link_libraries(offboard_center
  ${catkin_LIBRARIES}
)

add_executable(test1
  src/test.cpp
)

target_link_libraries(test1
  ${catkin_LIBRARIES}
)

add_executable(takeoff_test
  src/takeoff_test.cpp
)

target_link_libraries(takeoff_test
  ${catkin_LIBRARIES}
)
