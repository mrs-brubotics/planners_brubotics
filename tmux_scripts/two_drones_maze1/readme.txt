two drones Maze1: 

The UAV2 spawns in -9,-5, UAV1 spawns in 6,6 and this latter must go through the maze thx to a trajectory built with the planner using RRTConnect algorithm.

Experiments done with this script : 
- 2 UAVs in maze with static obstacle
- 2 UAVs in maze in maze without static obstacle

Parameters: 
- DSM_s, DSM_a and NF_a_nco/co always ON. DSM_o + NF_o_nco/co turned ON or OFF 
- Replanning rate : python_interface.py line 103 and 113 to activate and set the rate 
