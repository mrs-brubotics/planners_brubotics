# maps
This folder contains maps produced by scanning the environmment with a sensor (LIDAR, Depth camera) and generating Octomaps from its pointcloud2 topic. 

To produce one : 
- Use the one_drone_mapping session with your premade world file you want to map 
- plan a trajectory to map step by step yor world
- When you want to save your map, run this line of code WHILE THE OCTOMAP IS RUNNING : rosrun octomap_server octomap_saver -f (yourname).bt 

To load it in your session :

- rosrun octomap_server octomap_server_node Maze2.bt


