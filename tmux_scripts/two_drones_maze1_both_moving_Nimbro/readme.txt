This test validates the agent avoidance part of the D-ERG under RTK odometry using the Nimbro communication. If you do not have the Nimbro set up but want to test the agent avoidance part of the D-ERG, use script 6_two_drones_DERG_agent_avoidance_No_Nimbro.

You need at least 2 different computers to run this experiment. 2 simulations will be created and both uavs will communicate with each other and avoid each other.
The tracker of type DergbryanTracker is always enabled and should not be changed.
The controller after takeoff is set to the Se3BruboticsController and should not be changed.
One drone (name depends on the machine) is spawned at the position defined in spawn_location.yaml. 

How to configure this test?
1) Configure the following files only where you find "[USER INPUT]"
    - custom_configs/dergbryan_tracker : Depending on the behaviour you want to have, change the influence_margin, static_safety_margin, circulation gain, and DSM parameters kappa_a_
    - motor_params.yaml: Select the motor parameters according to if you are flying in simulation or with hardware.
    - gains/se3_brubotics.yaml: Change the value of thrust_saturation depending on if you are doing simulations or hardware experiments.
    - session.yaml: In the start_trajectory tab, either load steps_uav1.txt (trajectory of uav1), steps_uav2.txt (trajectory of uav2) or steps_uav4.txt (uav is at [0,0,2] and does not move).
2) Run ./start.sh
3) Wait until the drone with UAV_NAME has taken off
4) Command the drone from the "start_trajectory" tab and wait for the dynamics to be damped out completely.

5a) EXPERIMENT 1: For this you need to load steps_uav1.txt on one computer and steps_uav3.txt on the other computer. Go to the "rosbag_start_challenge" tab and simultaneously start the trajectory task + rosbag logging. You can just launch a command from 1 tab, the other will be performed as well. The agent constraint test will then be performed: 2 uavs are spawned at coordinates that are given in the spawn_location.yaml file of the different computers. Then, the 2 uavs are asked to switch positions. This will have as consequence that both uavs will move on the same line towards each other, until the repulsion field of the different agents makes them avoid each other.

5b) EXPERIMENT 2: For this you need to load steps_uav3.txt on one computer and steps_uav4.txt on the other computer. Go to the "rosbag_start_challenge" tab and simultaneously start the trajectory task + rosbag logging. You can just launch a command from 1 tab, the other will be performed as well. The agent constraint test will then be performed: 2 uavs are spawned at coordinates that are given in the spawn_location.yaml file of the different computers. Then, the 1 uav is asked to follow a trajectory where the other uav is in its way. This will have as consequence that uav will avoid the static uav the same way it avoids obstacles, due to the agent repulsion field.

6) Press ctrl + c to stop rosbag logging when the 2 drones stop moving due when they reached their respective destinations (wait for the dynamics to be damped out completely).
7) Press Ctrl-a + k to kill the tmux session (or tmux kill-server).
8) Plot the logged data using the local MatLab scripts in the results folder (AgentAvoidanceUAV1.mat and AgentAvoidanceUAV3.mat or AgentAvoidanceTest2UAV1.mat and AgentAvoidanceTest2UAV3.mat) and in the generic_matlab_plots/ThesisA. The results should look like the images you see in the results folder.


