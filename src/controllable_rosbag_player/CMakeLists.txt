cmake_minimum_required(VERSION 2.8.3)
project(controllable_rosbag_player)

find_package(catkin REQUIRED COMPONENTS
        rospy
        std_srvs
        message_generation
        )

### Generate services
add_service_files(
        FILES
        Seek.srv
        SetPlaybackSpeed.srv
)
generate_messages()

###################################
## catkin specific configuration ##
###################################
catkin_package(
        CATKIN_DEPENDS rospy std_srvs message_runtime
)

###########
## Build ##
###########
SET(CMAKE_CXX_FLAGS "-std=c++11 -O2 -g -Wall ${CMAKE_CXX_FLAGS}")

install(DIRECTORY launch/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
        PATTERN ".svn" EXCLUDE)
