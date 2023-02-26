# ROS-Navigation-Projects
## Lab1: Obstacle avoidance and using tf in ROS

### Part 1:
This part focuses on controlling the robot to avoid obstacles based on the laser sensor readings moving with a speed of 2m/sec.

### Part2:
In the part the pursuer and evader setup is created in which the coordinate frame of the evader are published with respect to the global frame, a new world is created with the pursuer and dropped in to the world close to the evader, the pursuer subscribes to the tf messages from the evader and follows the evader by going to the spot it was at from one second before.A third launch file is created pursuer-evader.launch to run the new world with the two robots, the evader controller, and the pursuer controller as separate nodes.

