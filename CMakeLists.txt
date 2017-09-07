cmake_minimum_required(VERSION 2.8.3)
project(utm_agents)
SET(CMAKE_CXX_FLAGS "-std=c++1y")

set( LIBS_TO_LINK yaml-cpp )

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  rospy
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  #LIBRARIES utm_agents yaml-cpp
  CATKIN_DEPENDS geometry_msgs nav_msgs roscpp rospy std_msgs # agent_msgs
  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(utm_agents src/utm_agents.cpp)
target_link_libraries(utm_agents ${catkin_LIBRARIES} ${LIBS_TO_LINK})
add_dependencies(utm_agents ${catkin_EXPORTED_TARGETS})

add_executable(utm_manager src/utm_manager.cpp)
target_link_libraries(utm_manager ${catkin_LIBRARIES})
add_dependencies(utm_manager ${catkin_EXPORTED_TARGETS})
