# Global Planner for Quadcopter
This is a C++ based RRT implementation in ROS using [Octomap](http://wiki.ros.org/octomap). It is mainly used for quadcopter waypoint planning.

### Install:
Tested on Ubuntu 18.04, 20.04 with ROS melodic/Noetic. Current version depends on [Octomap](http://wiki.ros.org/octomap).
```
cd ~/catkin_ws/src
git clone https://github.com/Zhefan-Xu/global_planner.git
cd ~/catkin_ws
catkin_make
```
### Run Planner DEMO:
Start the simulator by running:
```
roslaunch global_planner rrtInteractive.launch
```
Use ```2D Nav Goal``` in ```Rviz``` to select start and goal position in the map.

![Screenshot from 2022-01-21 21-50-47](https://user-images.githubusercontent.com/55560905/150622335-b3bdd8ef-c836-48cf-bafb-a9895a7dc1d9.png)

### Parameters:
RRT planner paramters can be changed in ```global_planner/cfg/planner.yaml```. The followings are the default values: 
- ```collision_box: [0.4, 0.4, 0.4]```
- ```env_box: [-100, 100, -100, 100, 0, 1.5]```
- ```timeout: 2.0```
- ```rrt_incremental_distance: 0.3```
- ```rrt_connect_goal_ratio: 0.2```
- ```goal_reach_distance: 0.4```
- ```map_resolution: 0.2```
- ```vis_RRT: False```
- ```vis_path: True```

### Code Exmaple & API:
Please see example ```global_planner/src/rrtInteractiveNode```. The example shows how to set start and goal position, and also how to find path. 

Note: the planner needs to call octomap service ```octomap_binary```, make sure to turn on your ```octomap server```.
