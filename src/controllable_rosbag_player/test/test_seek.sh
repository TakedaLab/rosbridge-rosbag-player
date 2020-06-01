#!/bin/bash

function usage() {
    echo "Usage: $0 <target timestamp>"
    echo "    <target timestamp>: Timestamp to seek"
}

[[ $# -ne 1 ]] && usage && exit 1

target=${1}

python <<EOF

from controllable_rosbag_player.srv import Seek
import rospy

service_name = '/rosbag_player_controller/seek'

print('Waiting for service "{}"'.format(service_name))
rospy.wait_for_service(service_name)
print('Calling service "{}"'.format(service_name))
try:
    seek = rospy.ServiceProxy(service_name, Seek)
    res = seek(${target})
    print(res)
except rospy.ServiceException, e:
    print("Service call failed: {}".format(str(e)))

EOF
