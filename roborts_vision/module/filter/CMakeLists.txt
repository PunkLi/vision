project(filter)

add_library(${PROJECT_NAME}_LIBS
  Filter.cpp
)

target_link_libraries(${PROJECT_NAME}_LIBS
  PUBLIC
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)