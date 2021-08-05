cmake_minimum_required(VERSION 2.8.3)
project(sprint)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  std_msgs
  aruku)

catkin_package(
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp std_msgs sensor_msgs aruku)

include_directories(
  ${catkin_INCLUDE_DIRS})

add_executable(${PROJECT_NAME}_main src/sprint_main.cpp)
add_dependencies(${PROJECT_NAME}_main ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_main ${catkin_LIBRARIES})

install(TARGETS ${PROJECT_NAME}_main
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
