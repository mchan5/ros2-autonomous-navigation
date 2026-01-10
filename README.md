# ROS2 Autonomous Navigation
![ROS 2](https://img.shields.io/badge/ROS2-Humble-22314E?logo=ros&logoColor=white)
![C++](https://img.shields.io/badge/Language-C++17-00599C?logo=c%2B%2B&logoColor=white)

**An autonomous navigation system built in C++ and ROS 2, and simultaed in Foxglove.**

https://github.com/user-attachments/assets/6f316b25-a6cb-44a0-a53a-f0222cc9ac8e

# Process

### Control Node
* **Subscribers:** `/path`, `/odom`
* **Publishers:** `/cmd_vel`
* **Function:** Starts the journey by reorienting itself towards the final goal. It uses a **Pure Pursuit controller**, calculating the curvature required to reach a specific lookahead point, and dynamically throttles velocity based on turn sharpness to ensure a stable route.

### Costmap Node
* **Subscribers:** `/lidar`
* **Publishers:** `/costmap`
* **Function:** Calculates a "cost" for each section of the local area.
    * **Low Cost:** Unknown or safe to travel.
    * **High Cost:** Objects present; robot must avoid.
* **Logic:** It converts Lidar data (polar coordinates) into Cartesian points via trigonometry, marks obstacles, and **inflates** them so that surrounding cells receive a decaying cost based on distance from the obstacle.

### Map Memory Node
* **Subscribers:** `/costmap`, `/odom`
* **Publishers:** `/map`
* **Function:** Transforms local sensor data into the global frame and accumulates it into a consistent, persistent grid (often visualized in red).
* **Logic:** This ensures sensor data is "remembered" even after the robot moves away, creating a static map for the planner to refer to.

### Planner Node
* **Subscribers:** `/map`, `/goal_point`, `/odom`
* **Publishers:** `/path`
* **Function:** Implements the **A* (A-Star) algorithm** to find an efficient path to the goal point while considering obstacles.
* **Logic:** It features a **"Lazy Check" loop** that continuously validates if the *current* path is safe as new Lidar data is provided, rather than replanning every single frame.


# Acknowledgements
The framework and setup for this project was done by the WATonomous design team. The integration and programming of the modules listed above were done by me.
