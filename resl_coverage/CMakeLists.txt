cmake_minimum_required(VERSION 2.8.3)
project(resl_coverage)

# set(CMAKE_BUILD_TYPE Debug)

find_package(rosflight_firmware)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

# if(NOT rosflight_firmware_FOUND)
#   message(WARNING "Could not find rosflight_firmware package, skipping ${PROJECT_NAME}")
#   find_package(catkin)
#   catkin_package()
#   return()
# endif()


set(PKG_DEPENDS
  arl_unity_ros_air
  arl_unity_ros
  rosflight_firmware
  rosflight_msgs
  rosflight_utils
  )
set(BUILD_DEPENDS
  ${PKG_DEPENDS}
  )
set(RUN_DEPENDS
  ${PKG_DEPENDS}
  )

set(AURA ../../arl-unity-ros/arl_unity_ros_air)

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

add_executable(unity_rosflight ${AURA}/src/unity_rosflight.cpp ${AURA}/src/mavlink_board.cpp)
target_link_libraries(unity_rosflight ${catkin_LIBRARIES})

install(TARGETS unity_rosflight
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
  )

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  )

install(PROGRAMS ${AURA}/scripts/rc_sim ${AURA}/scripts/set_params.sh
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )

#catkin_install_python(PROGRAMS scripts/calibrate_imu.py scripts/rosflight_offboard.py
catkin_install_python(PROGRAMS ${AURA}/scripts/calibrate_imu.py ${AURA}/scripts/rosflight_offboard.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  )

install(DIRECTORY examples
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  )
