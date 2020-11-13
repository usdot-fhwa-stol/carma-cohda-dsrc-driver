cmake_minimum_required(VERSION 2.8.3)
project(dsrc_driver)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)
set( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall")
set( CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
set(ROS_DEPS
  cav_driver_utils
  cav_msgs
  cav_srvs
  sensor_msgs
  geometry_msgs
  bondcpp
  roscpp
  dynamic_reconfigure
)
find_package(catkin REQUIRED COMPONENTS
  ${ROS_DEPS}
)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system thread)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## Generate dynamic reconfigure parameters in the 'config' folder
 generate_dynamic_reconfigure_options(
	 config/DSRC.cfg
 )


###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  LIBRARIES dsrc_driver_library
  CATKIN_DEPENDS ${ROS_DEPS}
  DEPENDS Boost
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
 include
 thirdparty/rapidjson/include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

add_library(dsrc_driver_library 
  src/dsrc_application.cpp 
  src/dsrc_client.cpp
)
target_link_libraries(dsrc_driver_library ${Boost_LIBRARIES} ${catkin_LIBRARIES})
add_dependencies(dsrc_driver_library ${catkin_EXPORTED_TARGETS})


add_executable(${PROJECT_NAME}_node
                src/main.cpp
                src/dsrc_client.cpp
                src/dsrc_application.cpp)


target_link_libraries(${PROJECT_NAME}_node dsrc_driver_library ${Boost_LIBRARIES} ${catkin_LIBRARIES})
add_dependencies(${PROJECT_NAME}_node ${PROJECT_NAME}_gencfg ${catkin_EXPORTED_TARGETS})

#############
## Install ##
#############

## Mark executables and/or libraries for installation
install(TARGETS ${PROJECT_NAME}_node dsrc_driver_library
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY
  thirdparty
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

## Mark other files for installation (e.g. launch and bag files, etc.)
install(DIRECTORY
  etc
  launch
  config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
 catkin_add_gmock(${PROJECT_NAME}-test 
    test/test_dsrc_client.cpp
    WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/test)

 if(TARGET ${PROJECT_NAME}-test)
   target_link_libraries(${PROJECT_NAME}-test dsrc_driver_library  ${Boost_LIBRARIES} ${catkin_LIBRARIES})
 endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)

