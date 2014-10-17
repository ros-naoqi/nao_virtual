cmake_minimum_required(VERSION 2.8.3)
project(nao_gazebo_plugin)

find_package(catkin REQUIRED)

catkin_package()

install(DIRECTORY config launch worlds
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
