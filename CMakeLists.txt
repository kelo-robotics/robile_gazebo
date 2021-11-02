cmake_minimum_required(VERSION 3.5.1)
project(robile_gazebo)

find_package(catkin REQUIRED)

find_package(gazebo)
find_package(roslaunch)

catkin_package()

install(DIRECTORY config launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
