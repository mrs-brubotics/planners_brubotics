One drone Maze1: 

The drone spawns in 6,6 and must go through the maze thx to a trajectory built with the planner using RRTConnect algorithm.

Experiments done with this script : 
- UAV1 in maze with static obstacle
- UAV1 in maze without static obstacle

Parameters: 
- DSM s always ON. DSM_o + NF_o_nco/co turned ON or OFF 
- Replanning rate : python_interface.py line 103 and 113 to activate and set the rate 