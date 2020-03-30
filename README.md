# rosbridge-rosbag-player

A package for playing a rosbag with ROS-services for controlling the playback.

## Build docker image
### Basic image
```bash
$ docker build -t hdwlab/rosbridge-rosbag-player:base-master .

```

### Autoware installed image
```bash
$ docker build -t hdwlab/rosbridge-rosbag-player:autoware-master -f Dockerfile.autoware .

```

## Play rosbag
The following command will start a container with rosbridge_websocket, 
controllable-rosbag-layer and rosbridge-server. 
Please replace `<path to rosbag>` with a proper path.
```bash
$ docker run -it --rm \
  -p 9090:9090 \
  -v <path to rosbag>:/rosbag.bag:ro \
  hdwlab/rosbridge-rosbag-player:base-master \
  roslaunch /player.launch path_to_rosbag:=/rosbag.bag

```
A websocket will be available with `ws://localhost:9090`.  
Playback can be controlled via the following ros-services.

### Play rosbag
Call ROS-service `/rosbag_player_controller/play`  

### Pause rosbag
Call ROS-service `/rosbag_player_controller/pause`  

### Seek rosbag
Call ROS-service `/rosbag_player_controller/seek <start_time>`  

### Set playback speed
Call ROS-service `/rosbag_player_controller/set_playback_speed <speed>`  

