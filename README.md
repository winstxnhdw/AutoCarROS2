# AutoCarROS2: CONTROL AND BEHAVIOUR
### AutoCarROS has migrated to ROS 2 Foxy Fitzroy
<div align="center">
	<img src="https://github.com/winstxnhdw/AutoCarROS/blob/master/resources/pictures/ngeeann_av_ultrawide.png?raw=true" />
</div>

## Abstract
This project contains the ROS 2 variant of the AutoCarROS repository. It covers the development of a robust non-holonomic autonomous vehicle platform in a simulated environment using ROS and Gazebo. A sense-think-act cycle is implemented to navigate the virtual world, avoiding static and moving objects.
<br />
<br />
<div align="center">
	<img src="https://github.com/winstxnhdw/AutoCarROS/blob/master/resources/gifs/obstacle_avoidance.gif?raw=true" />
</div>

## Installation
After installing [ROS 2 Foxy Fitzroy](https://docs.ros.org/en/foxy/Installation.html),
```bash
# Clone repository
$ git clone https://github.com/winstxnhdw/AutoCarROS2.git
$ cd PATH/TO/WORKSPACE/src/AutoCarROS2

# Install additional dependencies
$ chmod +x requirements.sh
$ ./requirements.sh
```

## Usage
```bash
# Source workspace
$ cd PATH/TO/WORKSPACE/src/AutoCarROS2
$ source /opt/ros/foxy/setup.bash
$ . install/setup.bash

# Build packages
$ colcon build

# Launch
$ ros2 launch launches av_launch.py
```

## Renders
<b>"Because the layman doesn't care unless it looks cool."<b>
<br />
<br />
<div align="center">
	<img src="https://github.com/winstxnhdw/AutoCarROS/blob/master/resources/gifs/renders.gif?raw=true" />
	<img src="https://github.com/winstxnhdw/AutoCarROS/blob/master/resources/gifs/1.gif?raw=true" />
	<img src="https://github.com/winstxnhdw/AutoCarROS/blob/master/resources/gifs/2.gif?raw=true" />
	<img src="https://github.com/winstxnhdw/AutoCarROS/blob/master/resources/gifs/3.gif?raw=true" />
	<img src="https://github.com/winstxnhdw/AutoCarROS/blob/master/resources/gifs/4.gif?raw=true" />
</div>

