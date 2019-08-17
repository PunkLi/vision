project(detect_factory)

set(CMAKE_BUILD_TYPE Release)

find_package( OpenMP REQUIRED)  

if(OPENMP_FOUND)  
  message("OPENMP FOUND")  
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")  
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")  
  set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")  
endif()  

add_library(${PROJECT_NAME}_LIBS
  armor/RM2017/constraint_set.cpp
  armor/CC/constraint_set.cpp
  armor/HYY/constraint_set.cpp
  armor/LCP/constraint_set.cpp
  rune/Rune_fiall.cpp
)

target_link_libraries(${PROJECT_NAME}_LIBS
  PUBLIC
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)