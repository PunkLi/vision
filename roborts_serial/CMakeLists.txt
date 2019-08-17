project(roborts_serial)
cmake_minimum_required(VERSION 3.1)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_BUILD_TYPE Release)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf
  roborts_msgs
  )

catkin_package()

include_directories(
  "sdk"
  ${catkin_INCLUDE_DIRS}
)

## main
add_executable(serial_node
  serial/main.cpp
  serial/serial_node.cpp
  sdk/serial_device.cpp
)

target_link_libraries(serial_node PUBLIC
  ${catkin_LIBRARIES})

target_include_directories(serial_node PUBLIC ${catkin_INCLUDE_DIRS})

add_dependencies(serial_node roborts_msgs_generate_messages)

## fake tf
add_executable(fake_serial
  test/fake_serial.cpp
)

target_link_libraries(fake_serial PUBLIC
  ${catkin_LIBRARIES})

target_include_directories(fake_serial PUBLIC ${catkin_INCLUDE_DIRS})

add_dependencies(fake_serial roborts_msgs_generate_messages)

## fake loop
add_executable(fake_loop
  test/fake_loop.cpp
)

target_link_libraries(fake_loop PUBLIC
  ${catkin_LIBRARIES})

target_include_directories(fake_loop PUBLIC ${catkin_INCLUDE_DIRS})

add_dependencies(fake_loop roborts_msgs_generate_messages)