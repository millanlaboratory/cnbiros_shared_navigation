set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
cmake_minimum_required(VERSION 2.8.3)
project(cnbiros_shared_navigation)
find_package(catkin REQUIRED COMPONENTS roscpp actionlib std_msgs nav_msgs message_generation grid_map_ros tf costmap_2d move_base move_base_msgs  grid_map_ros)

SET(CMAKE_BUILD_TYPE RelWithDebInfo)

include_directories(include ${catkin_INCLUDE_DIRS})

#add dynamic reconfigure api
#find_package(catkin REQUIRED dynamic_reconfigure)
generate_dynamic_reconfigure_options(
	cfg/CostMapSector.cfg
	cfg/PointSector.cfg
	cfg/SharedActions.cfg
  #...
)

add_message_files(
	FILES
	ProximitySectorMsg.msg
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
	move_base
	move_base_msgs
	grid_map_ros
)

add_library(cnbiros_shared_navigation 
			src/ProximitySector.cpp
			src/ProximitySectorConverter.cpp
			src/CostMapToProximitySector.cpp
			src/PointToProximitySector.cpp
			src/SharedActions.cpp)
add_dependencies(cnbiros_shared_navigation cnbiros_shared_navigation_generate_messages_cpp)
target_link_libraries(cnbiros_shared_navigation ${catkin_LIBRARIES})


add_executable(costmap_to_proximitysector src/costmap_to_proximitysector.cpp)
target_link_libraries(costmap_to_proximitysector cnbiros_shared_navigation ${catkin_LIBRARIES})
add_dependencies(costmap_to_proximitysector ${PROJECT_NAME}_gencfg)

add_executable(point_to_proximitysector src/point_to_proximitysector.cpp)
target_link_libraries(point_to_proximitysector cnbiros_shared_navigation ${catkin_LIBRARIES})
add_dependencies(point_to_proximitysector ${PROJECT_NAME}_gencfg)

add_executable(shared_actions src/shared_actions.cpp)
target_link_libraries(shared_actions cnbiros_shared_navigation ${catkin_LIBRARIES})
add_dependencies(shared_actions ${PROJECT_NAME}_gencfg)

#add_executable(user_goal src/user_goal.cpp)
#target_link_libraries(user_goal cnbiros_shared_navigation ${catkin_LIBRARIES})
#
#add_executable(simple_navigation_goal src/simple_navigation_goal.cpp)
#target_link_libraries(simple_navigation_goal ${catkin_LIBRARIES})

