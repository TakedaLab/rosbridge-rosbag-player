#!/bin/bash

python <<EOF

from std_srvs.srv import Trigger
import rospy

service_name = '/rosbag_player_controller/pause'

print('Waiting for service "{}"'.format(service_name))
rospy.wait_for_service(service_name)
print('Calling service "{}"'.format(service_name))
try:
    proxy = rospy.ServiceProxy(service_name, Trigger)
    res = proxy()
    print(res)
except rospy.ServiceException, e:
    print("Service call failed: {}".format(str(e)))

EOF
