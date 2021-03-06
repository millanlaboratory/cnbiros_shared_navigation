set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
cmake_minimum_required(VERSION 2.8.3)
project(cnbiros_shared_navigation)
find_package(catkin REQUIRED COMPONENTS roscpp actionlib std_msgs nav_msgs message_generation grid_map_ros tf costmap_2d move_base_msgs  grid_map_ros)

SET(CMAKE_BUILD_TYPE RelWithDebInfo)

include_directories(include ${catkin_INCLUDE_DIRS})

#add dynamic reconfigure api
#find_package(catkin REQUIRED dynamic_reconfigure)
generate_dynamic_reconfigure_options(
	cfg/CostMapSector.cfg
	cfg/PointSector.cfg
	cfg/LaserScanSector.cfg
	cfg/LaserScanGrid.cfg
	cfg/PointGrid.cfg
	cfg/SharedActions.cfg
	cfg/SharedDynamics.cfg
  #...
)

add_message_files(
	FILES
	ProximitySectorMsg.msg
	ProximityGridMsg.msg
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
	move_base_msgs
	grid_map_ros
)

add_library(cnbiros_shared_navigation 
			src/ProximitySector.cpp
			src/ProximitySectorConverter.cpp
			src/CostMapToProximitySector.cpp
			src/PointToProximitySector.cpp
			src/LaserScanToProximitySector.cpp
			src/ProximityGrid.cpp
			src/ProximityGridConverter.cpp
			src/LaserScanToProximityGrid.cpp
			src/PointToProximityGrid.cpp
			src/ProximityGridVisualizer.cpp
			src/SharedActions.cpp
			src/SharedDynamics.cpp)
add_dependencies(cnbiros_shared_navigation cnbiros_shared_navigation_generate_messages_cpp)
target_link_libraries(cnbiros_shared_navigation ${catkin_LIBRARIES})


add_executable(costmap_to_proximitysector src/costmap_to_proximitysector.cpp)
target_link_libraries(costmap_to_proximitysector cnbiros_shared_navigation ${catkin_LIBRARIES})
add_dependencies(costmap_to_proximitysector ${PROJECT_NAME}_gencfg)

add_executable(point_to_proximitysector src/point_to_proximitysector.cpp)
target_link_libraries(point_to_proximitysector cnbiros_shared_navigation ${catkin_LIBRARIES})
add_dependencies(point_to_proximitysector ${PROJECT_NAME}_gencfg)

add_executable(laserscan_to_proximitysector src/laserscan_to_proximitysector.cpp)
target_link_libraries(laserscan_to_proximitysector cnbiros_shared_navigation ${catkin_LIBRARIES})
add_dependencies(laserscan_to_proximitysector ${PROJECT_NAME}_gencfg)

add_executable(laserscan_to_proximitygrid src/laserscan_to_proximitygrid.cpp)
target_link_libraries(laserscan_to_proximitygrid cnbiros_shared_navigation ${catkin_LIBRARIES})
add_dependencies(laserscan_to_proximitygrid ${PROJECT_NAME}_gencfg)

add_executable(point_to_proximitygrid src/point_to_proximitygrid.cpp)
target_link_libraries(point_to_proximitygrid cnbiros_shared_navigation ${catkin_LIBRARIES})
add_dependencies(point_to_proximitygrid ${PROJECT_NAME}_gencfg)

add_executable(proximitygrid_visualizer src/proximitygrid_visualizer.cpp)
target_link_libraries(proximitygrid_visualizer cnbiros_shared_navigation ${catkin_LIBRARIES})

add_executable(shared_actions src/shared_actions.cpp)
target_link_libraries(shared_actions cnbiros_shared_navigation ${catkin_LIBRARIES})
add_dependencies(shared_actions ${PROJECT_NAME}_gencfg)

add_executable(shared_dynamics src/shared_dynamics.cpp)
target_link_libraries(shared_dynamics cnbiros_shared_navigation ${catkin_LIBRARIES})
add_dependencies(shared_dynamics ${PROJECT_NAME}_gencfg)

add_executable(keyboard_goal src/keyboard_goal.cpp)
target_link_libraries(keyboard_goal cnbiros_shared_navigation ${catkin_LIBRARIES})

add_executable(simple_navigation_goal src/simple_navigation_goal.cpp)
target_link_libraries(simple_navigation_goal ${catkin_LIBRARIES})

