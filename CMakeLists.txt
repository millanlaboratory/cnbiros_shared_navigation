set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
cmake_minimum_required(VERSION 2.8.3)
project(cnbiros_wheelchair_navigation)

find_package(catkin REQUIRED COMPONENTS roscpp actionlib std_msgs nav_msgs message_generation grid_map_ros)

SET(CMAKE_BUILD_TYPE RelWithDebInfo)

include_directories(include ${catkin_INCLUDE_DIRS})

add_message_files(
	FILES
	PolarGrid.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  nav_msgs
)


catkin_package(
  INCLUDE_DIRS 
	include
  LIBRARIES 
	${PROJECT_NAME}
  CATKIN_DEPENDS
  	roscpp
	nav_msgs
)

add_library(cnbiros_wheelchair_navigation 
			src/CostMapToPolarGrid.cpp)
add_dependencies(cnbiros_wheelchair_navigation cnbiros_wheelchair_navigation_generate_messages_cpp)
target_link_libraries(cnbiros_wheelchair_navigation ${catkin_LIBRARIES})

add_executable(costmap_to_polargrid src/costmap_to_polargrid.cpp)
target_link_libraries(costmap_to_polargrid cnbiros_wheelchair_navigation ${catkin_LIBRARIES})

add_executable(simple_navigation_goal src/simple_navigation_goal.cpp)
target_link_libraries(simple_navigation_goal ${catkin_LIBRARIES})

