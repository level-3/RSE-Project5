# RSE-Project5
***Udacity Robotics Software Engineer Nanodegree** - *Project 5* - Home Service Robot*

Goal of this project is to  create a map of a simulated gazebo environment and autonomously navigate to markers using SLAM (simultaneous location and mapping) and view paths, markers and vizualise maps in rviz

*Packages used:*
- **gmapping** : perform SLAM and build maps of an environment with a laser range finder or RGBD equipped robot. create a 2-D occupancy grid map from laser and pose data collected by a mobile robot. (http://wiki.ros.org/gmapping) 
- **AMCL**: probabilistic localization system for a robot moving in 2D. It implements the adaptive (or KLD-sampling) Monte Carlo localization approach which uses a particle filter to track the pose of a robot against a known map. (http://wiki.ros.org/amcl)
- **ROS Navigation Stack**: 2D navigation stack that takes in information from odometry, sensor streams, and a goal pose and outputs safe velocity commands that are sent to a mobile base. (http://wiki.ros.org/navigation). Uses Dijkstra's algorithm, which is a variant of Uniform Cost Search to plan the robots trajectory to avoid obstacles.

:::image type="content" source="Screenshot.png" alt-text="Screenshot":::
