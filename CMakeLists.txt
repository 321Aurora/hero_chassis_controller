cmake_minimum_required(VERSION 3.10)
project(hero_chassis_controller)

## Use C++14
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

## By adding -Wall and -Werror, the compiler does not ignore warnings anymore,
## enforcing cleaner code.
#add_definitions(-Wall -Werror)

## Find catkin macros and libraries
find_package(catkin REQUIRED  COMPONENTS
    roscpp
    control_toolbox
    roslint
    controller_interface
    hardware_interface
    forward_command_controller
    pluginlib
    geometry_msgs
    nav_msgs
    std_msgs
    tf2
    tf2_ros
    tf
    )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
    INCLUDE_DIRS
    include
    ## This is only necessary because Eigen3 sets a non-standard EIGEN3_INCLUDE_DIR variable
    LIBRARIES
    CATKIN_DEPENDS
    roscpp
    tf2
    tf2_ros
    control_toolbox
    roslint
    controller_interface
    hardware_interface
    forward_command_controller
    pluginlib
    DEPENDS
    tf
    std_msgs
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  #  include
    ${catkin_INCLUDE_DIRS}
    # Set manually because Eigen sets a non standard INCLUDE DIR
)

## Declare cpp executables
#add_library(${PROJECT_NAME}
 #   src/${PROJECT_NAME}.cpp
  #  )

#add_dependencies(${PROJECT_NAME}
 #   ${catkin_EXPORTED_TARGETS}
  #  )

#target_link_libraries(${PROJECT_NAME}
 #   ${catkin_LIBRARIES}
  #  )

#############
## Install ##
#############

## Mark executables and/or libraries for installation
#install(
#    TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_core
#    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#)
#
## Mark cpp header files for installation
#install(
#    DIRECTORY include/${PROJECT_NAME}/
#    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#    FILES_MATCHING PATTERN "*.hpp"
#)
#
## Mark other files for installation
#install(
#    DIRECTORY doc
#    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
#)
#
##############
### Testing ##
##############
#
#if (${CATKIN_ENABLE_TESTING})
#  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")
#  ## Add gtest based cpp test target and link libraries
#  catkin_add_gtest(${PROJECT_NAME}-test
#      test/test_ros_package_template.cpp
#      test/AlgorithmTest.cpp)
#  target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME}_core)
#endif ()

##########################
## Static code analysis ##
##########################


add_executable(pid_control ./src/pid_control.cpp)
add_executable(wheel_speed_calculator ./src/wheel_speed_calculator.cpp)
add_executable(odometry_publisher ./src/odometry_publisher.cpp)
 add_executable(tf_transformer ./src/tf_transformer.cpp)
target_link_libraries(pid_control ${catkin_LIBRARIES})
target_link_libraries(wheel_speed_calculator ${catkin_LIBRARIES})
target_link_libraries(odometry_publisher ${catkin_LIBRARIES} )
 target_link_libraries(tf_transformer ${catkin_LIBRARIES})

target_link_libraries(pid_control ${catkin_LIBRARIES}  yaml-cpp)





roslint_cpp()
