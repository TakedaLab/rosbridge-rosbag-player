#!/usr/bin/env python
#
# Controllable rosbag player
#
# Copyright 2020 Human Dataware Lab. Co. Ltd.
# Created by Daiki Hayashi (hayashi.daiki@hdwlab.co.jp)
#

import os
import signal
import subprocess
import sys
import threading
import time

import rosbag
import rospy

from controllable_rosbag_player.srv import Seek, SeekResponse, SetPlaybackSpeed, SetPlaybackSpeedResponse
from std_srvs.srv import SetBool, Trigger, TriggerResponse
from rosgraph_msgs.msg import Clock


class ControllableRosbagPlayer(object):
    """Controllable rosbag player."""

    def __init__(self):
        """Initialize ControllableRosbagPlayer."""
        super(ControllableRosbagPlayer, self).__init__()

        # initialize node
        self.player_node_name = 'rosbag_player'
        self.is_playing = False
        self.playback_speed = 1.0
        self.rosbag_start_time = 0.0
        self.rosbag_end_time = 0.0

        # initialize publishers
        self.start_time_publisher = rospy.Publisher('~rosbag_start_time', Clock, queue_size=1)
        self.end_time_publisher = rospy.Publisher('~rosbag_end_time', Clock, queue_size=1)

        # get ros-param
        self.path_to_rosbag = rospy.get_param('~path_to_rosbag', None)
        if self.path_to_rosbag is None:
            raise ValueError('rosbag path is not specified')

        # set common variables
        self.common_options = '--clock --keep-alive __name:={} '.format(self.player_node_name)
        self.service_address = '/{}/pause_playback'.format(self.player_node_name)

        # process
        self.subprocess = None
        self.subprocess_watcher = None

        # analyze rosbag
        self._analyze_rosbag(self.path_to_rosbag)

        # start publisher
        self.rosbag_info_publisher = threading.Thread(target=self.publish_rosbag_info)
        self.rosbag_info_publisher.setDaemon(True)
        self.rosbag_info_publisher.start()

        # open rosbag
        self._open_rosbag()

    def _analyze_rosbag(self, path_to_rosbag):
        with rosbag.Bag(path_to_rosbag, 'r') as bag:
            self.rosbag_start_time = bag.get_start_time()
            self.rosbag_end_time = bag.get_end_time()

    def _open_rosbag(self):
        command = 'rosrun rosbag play {0} {1} --pause'.format(
            ' '.join(self.path_to_rosbag.split(':')),
            self.common_options
        )
        self.subprocess = subprocess.Popen(command, shell=True)
        rospy.logdebug('Sent command: {}'.format(command))
        self.subprocess_watcher = threading.Thread(target=self.watch_subprocess)
        self.subprocess_watcher.setDaemon(True)
        self.subprocess_watcher.start()

    def pause(self):
        """Pause rosbag-player"""

        rospy.wait_for_service(self.service_address)
        try:
            pauser = rospy.ServiceProxy(self.service_address, SetBool)
            pauser(True)
            self.is_playing = False
            return True
        except rospy.ServiceException as e:
            rospy.logerr(e)
            return False
        except Exception as e:
            rospy.logerr(e)
            return False

    def play(self):
        """Play rosbag-player"""

        rospy.wait_for_service(self.service_address)
        try:
            pauser = rospy.ServiceProxy(self.service_address, SetBool)
            pauser(False)
            self.is_playing = True
            return True
        except rospy.ServiceException as e:
            rospy.logerr(e)
            return False
        except Exception as e:
            rospy.logerr(e)
            return False

    def seek(self, start_time):
        """Seek rosbag-player.

        Args:
            start_time (float): start time to seek

        Returns:
            (bool): True if succeeded, otherwise False

        """
        if start_time + self.rosbag_start_time > self.rosbag_end_time:
            return False
        try:
            command = 'rosrun rosbag play {0} -s {1} -r {2} {3}'.\
                format(' '.join(self.path_to_rosbag.split(':')),
                       start_time,
                       self.playback_speed,
                       self.common_options)
            if not self.is_playing:
                command += ' --pause'
            self.subprocess = subprocess.Popen(command, shell=True)
            rospy.logdebug('Sent command: {}'.format(command))
            return True
        except Exception as e:
            return False

    def set_playback_speed(self, speed):
        """Set playback speed.

        Args:
            speed (float): playback-rate

        Returns:
            (bool): True if succeeded, otherwise False

        """
        current_time = rospy.Time.now().to_sec()
        if current_time < self.rosbag_start_time or current_time > self.rosbag_end_time:
            target_time = 0.0
        else:
            target_time = current_time - self.rosbag_start_time
        try:
            command = 'rosrun rosbag play {0} -s {1} -r {2} {3}'. \
                format(' '.join(self.path_to_rosbag.split(':')),
                       target_time,
                       speed,
                       self.common_options)
            if not self.is_playing:
                command += ' --pause'
            self.subprocess = subprocess.Popen(command, shell=True)
            rospy.logdebug('Sent command: {}'.format(command))
            self.playback_speed = speed
            return True
        except Exception as e:
            return False

    def close(self):
        if self.subprocess is not None:
            self.subprocess.terminate()
        if self.subprocess.poll() is None:
            self.subprocess.kill()
        if self.subprocess_watcher is not None:
            rospy.logdebug('Watch thread alive, but it will die on program termination')

    def signal_handler(self, sig, frame):
        rospy.loginfo('Closing...')
        self.close()
        sys.exit(0)

    def watch_subprocess(self):
        fail_count = 0
        while not rospy.is_shutdown():
            time.sleep(1)
            if self.subprocess is None:
                continue
            if self.subprocess.poll() is not None:
                fail_count += 1
                rospy.logerr('Subprocess is not running (count: {})...'.format(fail_count))
                if fail_count == 3:
                    rospy.logerr('Restarting subprocess...')
                    if not self.set_playback_speed(speed=self.playback_speed):
                        rospy.logerr('Failed to restart subprocess')
                        os._exit(1)
            else:
                fail_count = 0

    def publish_rosbag_info(self):
        while not rospy.is_shutdown():
            rosbag_start_time = Clock()
            rosbag_start_time.clock.secs = int(self.rosbag_start_time)
            rosbag_start_time.clock.nsecs = int((self.rosbag_start_time % 1) * (10 ** 9))
            rosbag_end_time = Clock()
            rosbag_end_time.clock.secs = int(self.rosbag_end_time)
            rosbag_end_time.clock.nsecs = int((self.rosbag_end_time % 1) * (10 ** 9))
            self.start_time_publisher.publish(rosbag_start_time)
            self.end_time_publisher.publish(rosbag_end_time)
            time.sleep(0.1)


def callback_srv_play(data):
    """Play rosbag-player."""
    res = TriggerResponse()
    if player.play():
        res.message = 'succeed'
        res.success = True
    else:
        res.message = 'failed'
        res.success = False
    return res


def callback_srv_pause(data):
    """Pause rosbag-player."""
    res = TriggerResponse()
    if player.pause():
        res.message = 'succeed'
        res.success = True
    else:
        res.message = 'failed'
        res.success = False
    return res


def callback_srv_seek(data):
    """Seek rosbag-player."""
    res = SeekResponse()
    if player.seek(data.time):
        res.message = 'success'
        res.success = True
    else:
        res.message = 'failed'
        res.success = False
    return res


def callback_srv_set_playback_speed(data):
    """Set playback speed of rosbag-playback."""
    res = SetPlaybackSpeedResponse()
    if player.set_playback_speed(data.speed):
        res.message = 'success'
        res.success = True
    else:
        res.message = 'failed'
        res.success = False
    return res


if __name__ == '__main__':
    # initialize ROS node
    rospy.init_node('rosbag_player_controller',
                    log_level=rospy.WARN,
                    disable_signals=True)

    # create player object
    player = ControllableRosbagPlayer()

    # register services
    srv_play = rospy.Service('~play', Trigger, callback_srv_play)
    srv_pause = rospy.Service('~pause', Trigger, callback_srv_pause)
    srv_seek = rospy.Service('~seek', Seek, callback_srv_seek)
    srv_set_playback_speed = rospy.Service('~set_playback_speed',
                                           SetPlaybackSpeed,
                                           callback_srv_set_playback_speed)

    # register signal handler for killing
    signal.signal(signal.SIGINT, player.signal_handler)

    # spin
    rospy.spin()
