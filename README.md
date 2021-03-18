# planners_brubotics
![alt text](https://github.com/mrs-brubotics/planners_brubotics/blob/main/.fig/background.jpg)
## prerequisite
* Install MRS
* Install Moveit 1 - Melodic following [instruction](http://docs.ros.org/en/melodic/api/moveit_tutorials/html/doc/getting_started/getting_started.html)
## Quike Test 
This package is built based on the [mrs_uav_system](https://github.com/ctu-mrs/mrs_uav_system). 
* Run mrs: go ~/mrs_workspace/src/simulation/example_tmux_scripts/one_drone_gps, run:
```
./start.sh
```
* The sconde step under planners_brubotics workspace, run:
```
source devel/setup.bash
```
```
roslaunch moveit_planner execute.launch
```


