cmake_minimum_required(VERSION 2.8.3)
project(rosbridge_observer)

find_package(catkin REQUIRED COMPONENTS
        rospy
        rosbridge_msgs
        )

###################################
## catkin specific configuration ##
###################################
catkin_package(
        CATKIN_DEPENDS rospy rosbridge_msgs
)

###########
## Build ##
###########
SET(CMAKE_CXX_FLAGS "-std=c++11 -O2 -g -Wall ${CMAKE_CXX_FLAGS}")

install(DIRECTORY launch/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
        PATTERN ".svn" EXCLUDE)
