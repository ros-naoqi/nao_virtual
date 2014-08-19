cmake_minimum_required(VERSION 2.8.3)
project(nao_dcm_driver)

# Tell CMake where to find "FindNAOqi.cmake"
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  tf
  hardware_interface
  controller_manager
  nao_dcm_msgs
  sensor_msgs
)

find_package(NAOqi QUIET COMPONENTS
    alcommon
    alvision
    alproxies
    alerror
    alvalue
    qi
    qitype
)

find_package(Boost)

##Needed for ros packages
catkin_package(CATKIN_DEPENDS roscpp geometry_msgs tf std_msgs nao_dcm_msgs sensor_msgs hardware_interface controller_manager)

if( NAOqi_FOUND AND Boost_FOUND)
    message(STATUS "NAOqi C++ SDK found; C++ nodes will be built")
    include_directories( include
        ${catkin_INCLUDE_DIRS}
        ${NAOqi_INCLUDE_DIRS}
    	  ${Boost_INCLUDE_DIRS})
    add_executable(nao_dcm_driver src/nao_driver.cpp src/nao.cpp)
    target_link_libraries(nao_dcm_driver
        ${catkin_LIBRARIES}
        ${NAOqi_LIBRARIES}
        ${Boost_LIBRARIES})
    add_dependencies(nao_dcm_driver ${catkin_EXPORTED_TARGETS})
else()
    message(STATUS "Cannot find NAOqi C++ sdk; C++ nodes will NOT be built")
endif()


install(TARGETS nao_dcm_driver
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# install(DIRECTORY launch 
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
