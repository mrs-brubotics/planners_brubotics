# planners_brubotics (beta)
![alt text](https://github.com/mrs-brubotics/planners_brubotics/blob/main/.fig/background.jpg)
## Prerequisites
* [MRS](https://github.com/ctu-mrs/mrs_uav_system)
* Moveit 1 - Melodic, following [instruction](http://docs.ros.org/en/melodic/api/moveit_tutorials/html/doc/getting_started/getting_started.html)
* Numpy for python2.7: 
```
pip install numpy
```
## Quick Test 
* Run mrs: go ~/mrs_workspace/src/simulation/example_tmux_scripts/one_drone_gps, run:
```
./start.sh
```
* Go planners_brubotics workspace, run:
```
source devel/setup.bash
```
* Launch moveit planner:
```
roslaunch moveit_planner execute.launch
```
* Launch command interface (for testing):
```
roslaunch moveit_planner python_interface.launch
```



