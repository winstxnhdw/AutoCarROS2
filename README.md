# AutoCarROS2
### AutoCarROS has come to ROS2
![ngeeann_av](https://github.com/winstxnhdw/AutoCarROS/blob/master/resources/pictures/ngeeann_av_ultrawide.png?raw=true)

## Abstract
This project contains the ROS2 variant of the [AutoCarROS](https://github.com/winstxnhdw/AutoCarROS) repository. It covers the development of a robust non-holonomic autonomous vehicle platform in a simulated environment using ROS and Gazebo. A sense-think-act cycle is implemented to navigate the virtual world, avoiding static and moving objects.
<br />
<br />
![Obstacle Avoidance](https://github.com/winstxnhdw/AutoCarROS/blob/master/resources/gifs/obstacle_avoidance.gif?raw=true)

## Launch Files
|Launch File|Launches|Purpose|
|-----------|--------|-------|
|gazebo.launch|gazebo, no world, ngeeann_av|For debugging
|road.launch|gazebo, road.world, ngeeann_av|Foundational launch file for future launch files
|display.launch|rviz, ngeeann_av|Foundational launch file for future launch files
|controller.launch|axle controllers, steer controllers|Foundational launch file for future launch files

### ackermann_vehicle.launch
Launches the populated_road.world file into Gazebo and spawns the ngeeann_av onto a populated road world. It also launches ackermann_controller.launch, RViz, the controller spawner and ackermann controller. If your Gazebo does not start, this is because you do not have the required Gazebo models in your models folder. To fix this, you may change the ackermann_vehicle.launch parameters to launch the unpopulated road variant, road.launch.

### ackerman_controller.launch
Launches nodes used by both RViz and Gazebo when visualizing a vehicle with Ackermann steering.
## Renders
<b>"Because the layman doesn't care unless it looks cool."<b>
<br />
<br />
![Renders](https://github.com/winstxnhdw/AutoCarROS/blob/master/resources/gifs/renders.gif?raw=true)
![1](https://github.com/winstxnhdw/AutoCarROS/blob/master/resources/gifs/1.gif?raw=true)
![2](https://github.com/winstxnhdw/AutoCarROS/blob/master/resources/gifs/2.gif?raw=true)
![3](https://github.com/winstxnhdw/AutoCarROS/blob/master/resources/gifs/3.gif?raw=true)
![4](https://github.com/winstxnhdw/AutoCarROS/blob/master/resources/gifs/4.gif?raw=true)
