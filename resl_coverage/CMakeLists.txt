cmake_minimum_required(VERSION 2.8.3)
project(resl_coverage)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")


set(PKG_DEPENDS
  arl_unity_ros
  )
set(BUILD_DEPENDS
  ${PKG_DEPENDS}
  )
set(RUN_DEPENDS
  ${PKG_DEPENDS}
  )

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

#######################################
## Find dependencies
#######################################

find_package(catkin REQUIRED ${BUILD_DEPENDS}
	message_generation
)

find_package(Eigen3)

add_message_files(
	FILES
	StateEstimate.msg
	MultiStateEstimate.msg
	ProbMapData.msg
)

add_service_files(
	FILES
	Topology.srv
	ProcessNoise.srv
	Failure.srv
	TriggerFail.srv
	Coordinates.srv
	State.srv
	Neighbors.srv
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  )

#######################################
## Declare Catkin package and installation
#######################################

generate_messages(
	DEPENDENCIES
	std_msgs
	geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS ${RUN_DEPENDS}
  DEPENDS
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
  )

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  )

install(DIRECTORY config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  )

install(DIRECTORY examples
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  )
