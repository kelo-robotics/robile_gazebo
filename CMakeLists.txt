cmake_minimum_required(VERSION 3.10.2)
project(robile_gazebo)

find_package(catkin REQUIRED COMPONENTS roscpp rviz)

find_package(gazebo)
find_package(roslaunch)

catkin_package(
    INCLUDE_DIRS include
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)

add_executable(robile_gazebo
    src/KeloDrive.cpp
    src/Robile.cpp
    src/main.cpp
)

target_link_libraries(robile_gazebo
    ${catkin_LIBRARIES}
)

install(DIRECTORY config launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
