# ROS2 Autonomous Navigation
![ROS 2](https://img.shields.io/badge/ROS2-Humble-22314E?logo=ros&logoColor=white)
![C++](https://img.shields.io/badge/Language-C++17-00599C?logo=c%2B%2B&logoColor=white)
![Platform](https://img.shields.io/badge/Platform-Ubuntu%2024.04%20%7C%20WSL2-orange)
![License](https://img.shields.io/badge/License-MIT-green)


# Process
Control: 
Subscribers: /path, /odom 
Publishers: /cmd_vel

Costmap: 
Subscriber: /lidar 
Publisher: /costmap

The costmap calculates a "cost" for each section of the area where the car drives. A low cost indicates that it is unknown if there is an object there or not, but it is safe to travel through that area. A high cost would indicate places where objects are present, and the robot should avoid. 
The costmap considers the lidar data, and converts its detections into cartesian points through trigonometry, and then marks that detected obstacle to a high cost. It then inflates the obstacle so that cells within an inflation radius are assigned a cost as well, which decreases from their distance from the detected obstacle. 


Map Memory: 
Subscribers: /costmap, /odom 
Publishers: /map 


Planner: 
Subscribers: /map, /goal_point, /odom
Publishers: /path 

