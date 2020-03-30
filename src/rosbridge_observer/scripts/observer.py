#!/usr/bin/env python
#
# Observe messages from rosbridge and exit automatically
#
# Copyright 2020 Human Dataware Lab. Co. Ltd.
# Created by Daiki Hayashi (hayashi.daiki@hdwlab.co.jp)
#

import signal
import sys
import time

import rospy
from rosbridge_msgs.msg import ConnectedClients


class RosbridgeObserver(object):
    """Rosbridge observer."""

    def __init__(self, log_level=rospy.WARN):
        """Initialize RosbridgeObserver.

        Args:
            log_level (log_level): log-level

        """
        super(RosbridgeObserver, self).__init__()

        # initialize node
        rospy.init_node('rosbridge_observer', log_level=log_level)
        self.time_node_initialized = time.time()
        self.time_last_status_update = None
        self.num_connected_clients = 0

        # get ros-param
        self.target_topic = rospy.get_param('~topic_connected_clients', '/connected_clients')
        self.graceful_time = rospy.get_param('~graceful_time', 5)
        self.timeout = rospy.get_param('~timeout', 30)
        if self.target_topic is None:
            raise ValueError('target topic is not specified')

        # initialize subscriber
        self.subscriber = rospy.Subscriber(self.target_topic, ConnectedClients, self.callback, queue_size=100)

    def signal_handler(self, sig, frame):
        rospy.loginfo('Exiting...')
        sys.exit(0)

    def spin(self):
        """Spin."""
        while not rospy.is_shutdown():
            if self.time_last_status_update is None:
                # Check time elapsed from node initialization
                if (time.time() - self.time_node_initialized) > self.timeout:
                    rospy.logerr('ROSBridge-observer timeout')
                    sys.exit(0)
            else:
                if self.num_connected_clients == 0:
                    # Check time elapsed from the time rosbridge have connection
                    if (time.time() - self.time_last_status_update) > self.graceful_time:
                        rospy.logwarn('ROSBridge lost client connections, so exiting...')
                        sys.exit(0)
            time.sleep(1)

    def callback(self, msg):
        """Callback when a message of the target topic is received.

        Args:
            msg (rosbridge_msgs.msg.ConnectedClients): a message

        """
        if self.time_last_status_update is None and len(msg.clients) == 0:
            return
        self.time_last_status_update = time.time()
        self.num_connected_clients = len(msg.clients)


if __name__ == '__main__':
    # create observer object
    observer = RosbridgeObserver()

    # register signal handler for killing
    signal.signal(signal.SIGINT, observer.signal_handler)

    # spin
    observer.spin()
