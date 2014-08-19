cmake_minimum_required(VERSION 2.8.3)
project(nao_dcm_bringup)

## Find catkin macros and libraries
find_package(catkin REQUIRED)
find_package(Boost)

##Needed for ros packages
catkin_package(CATKIN_DEPENDS rospy nao_dcm_driver)

install(DIRECTORY launch 
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
