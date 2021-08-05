cmake_minimum_required(VERSION 2.8.3)
project(keisan)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME})

include_directories(
  include
  ${catkin_INCLUDE_DIRS})

add_library(${PROJECT_NAME} 
  src/${PROJECT_NAME}/algebra.cpp
  src/${PROJECT_NAME}/matrix.cpp
  src/${PROJECT_NAME}/point.cpp
  src/${PROJECT_NAME}/vector.cpp
  src/${PROJECT_NAME}/vector2.cpp)
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
