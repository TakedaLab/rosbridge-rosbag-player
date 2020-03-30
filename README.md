# ros-provider

A package for playing a rosbag with a REST-server/ROS-services for controlling the playback.

## Build docker image
### Basic image
```bash
$ docker build -t hdwlab/scenario-tool:ros-provider-base-latest .

```

### Autoware installed image
```bash
$ docker build -t hdwlab/scenario-tool:ros-provider-autoware-latest . \
  --build-arg USE_AUTOWARE=true

```

### METI-Pegasus installed image
```bash
$ docker build -t hdwlab/scenario-tool:ros-provider-meti-pegasus-latest . \
  --build-arg GITHUB_USER=hdl-service --build-arg GITHUB_PASS=<token> \
  --build-arg USE_AUTOWARE=true --build-arg USE_METI_PEGASUS=true

```
You have to replace `<token>` with a corresponding token.

## Play rosbag
The following command will start a container with controllable-rosbag-layer and rosbridge-server. 
Please replace `<path to rosbag>` with a proper path.
```bash
$ docker run -it --rm \
  -p 5000:5000 \
  -p 9090:9090 \
  -v <path to rosbag>:/rosbag.bag:ro \
  hdwlab/scenario-tool:ros-provider-base-latest

```
A websocket will be available with `ws://localhost:9090`. 
Playback will be controllable with the following URI.

### Play rosbag
http://localhost:5000/play  
or call ROS-service `/rosbag_player_controller/play`

### Pause rosbag
http://localhost:5000/pause
or call ROS-service `/rosbag_player_controller/pause`  

### Seek rosbag
http://localhost:5000/seek?start_time=[seek position in seconds]  
or call ROS-service `/rosbag_player_controller/seek <start_time>`

### Set playback speed
http://localhost:5000/set_playback_speed?speed=[playback speed]  
or call ROS-service `/rosbag_player_controller/set_playback_speed <speed>`
