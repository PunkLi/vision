project(roborts_vision)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake_module)
set(CMAKE_BUILD_TYPE Release)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf
  roslib
  roborts_msgs
)

find_package(OpenCV REQUIRED)

catkin_package(
  INCLUDE_DIRS
)

include_directories(
  "include"
  "module/camera"
  "module/detect_factory"
  "module/executor"
  "module/filter"
  ${OpenCV_INCLUDE_DIRECTORIES}
  ${catkin_INCLUDE_DIRS}
)

add_subdirectory(module/camera)
add_subdirectory(module/detect_factory)
add_subdirectory(module/executor)
add_subdirectory(module/filter)

add_executable(${PROJECT_NAME}_node
  node/VisionNode.cpp
  node/Serial.cpp
  node/Image.cpp
  node/decision.cpp
)

target_link_libraries(${PROJECT_NAME}_node
  PRIVATE
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  camera_LIBS
  detect_factory_LIBS
  executor_LIBS
  filter_LIBS
)

add_dependencies(${PROJECT_NAME}_node roborts_msgs_generate_messages)