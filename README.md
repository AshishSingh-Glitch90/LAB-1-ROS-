# ROS-Navigation-Projects
## Lab1: Obstacle avoidance and using tf in ROS

### Part 1:
This part focuses on controlling the robot to avoid obstacles based on the laser sensor readings moving with a speed of 2m/sec.

### Part2:
In the part the pursuer and evader setup is created in which the coordinate frame of the evader are published with respect to the global frame, a new world is created with the pursuer and dropped in to the world close to the evader, the pursuer subscribes to the tf messages from the evader and follows the evader by going to the spot it was at from one second before.A third launch file is created pursuer-evader.launch to run the new world with the two robots, the evader controller, and the pursuer controller as separate nodes.

## Lab2: Laser-Based Perception and Navigation with Obstacle Avoidance

### Part1: Perception Using Laser Range Finder
In this part the RANSAC algorithm is implemented to determine the walls using the sensor data obtained from the laser sensor, the detected obstacles/lines are visualized in RVIZ that would verify the reliability of the algorithm.

### Part2: Bug2 Algorithm implementation
Robot is made to start at (-8.0, -2.0) with the goal to reach (4.5, 9.0), navigating its way by avoiding obstacles using the output of the BUG2 Algorithm.



