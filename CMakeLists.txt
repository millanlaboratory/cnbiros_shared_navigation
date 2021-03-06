set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
cmake_minimum_required(VERSION 2.8.3)
project(cnbiros_shared_navigation)
find_package(catkin REQUIRED COMPONENTS roscpp std_msgs nav_msgs message_generation tf dynamic_reconfigure)


SET(CMAKE_BUILD_TYPE RelWithDebInfo)

include_directories(include ${catkin_INCLUDE_DIRS})

generate_dynamic_reconfigure_options(
	cfg/LaserScanGrid.cfg
	cfg/PointGrid.cfg
	cfg/SharedDynamics.cfg
  #...
)

add_message_files(
	FILES
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
)

add_library(cnbiros_shared_navigation 
			src/ProximityGrid.cpp
			src/ProximityGridConverter.cpp
			src/LaserScanToProximityGrid.cpp
			src/PointToProximityGrid.cpp
			src/ProximityGridVisualizer.cpp
			src/SharedDynamics.cpp)
add_dependencies(cnbiros_shared_navigation cnbiros_shared_navigation_generate_messages_cpp)
target_link_libraries(cnbiros_shared_navigation ${catkin_LIBRARIES})

add_executable(laserscan_to_proximitygrid src/laserscan_to_proximitygrid.cpp)
target_link_libraries(laserscan_to_proximitygrid cnbiros_shared_navigation ${catkin_LIBRARIES})
add_dependencies(laserscan_to_proximitygrid ${PROJECT_NAME}_gencfg)

add_executable(point_to_proximitygrid src/point_to_proximitygrid.cpp)
target_link_libraries(point_to_proximitygrid cnbiros_shared_navigation ${catkin_LIBRARIES})
add_dependencies(point_to_proximitygrid ${PROJECT_NAME}_gencfg)

add_executable(proximitygrid_visualizer src/proximitygrid_visualizer.cpp)
target_link_libraries(proximitygrid_visualizer cnbiros_shared_navigation ${catkin_LIBRARIES})

add_executable(shared_dynamics src/shared_dynamics.cpp)
target_link_libraries(shared_dynamics cnbiros_shared_navigation ${catkin_LIBRARIES})
add_dependencies(shared_dynamics ${PROJECT_NAME}_gencfg)

add_executable(keyboard_attractor src/keyboard_attractor.cpp)
target_link_libraries(keyboard_attractor cnbiros_shared_navigation ${catkin_LIBRARIES})
add_dependencies(keyboard_attractor ${PROJECT_NAME}_gencfg)

