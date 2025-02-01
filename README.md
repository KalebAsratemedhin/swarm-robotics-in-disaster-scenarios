# Swarm-Robotics-in-Disaster-scenarios

## Title: Swarm Robotics for Survivor Detection and Rescue

## Problem: 
Disasters (Plane crashes, Earthquakes, Floods …) often leave survivors scattered in
challenging environments, making traditional rescue operations slow and inefficient. There is a
need for an autonomous and scalable system to improve rescue speed and success rates.
Objective: Develop a swarm robotics system that uses computer vision and machine learning to
identify and assist survivors in diverse environments efficiently.

## Approach:
- Simulation: Create disaster scenarios (e.g., urban ruins, forests) in Gazebo with
survivors, debris, and obstacles.
- Vision and AI: Train machine-learning models to detect survivors using cameras and
thermal sensors.
- Swarm Coordination: Design algorithms for robots to communicate and collaborate
effectively.
- Testing: Evaluate the system’s accuracy and efficiency in simulated environments.
Applications:
- Fast, scalable search and rescue in various disaster scenarios.
- Integration with emergency response systems.
- Environmental monitoring and cleanup.
This project aims to revolutionize disaster response through autonomous, technology-driven
solutions.

## How to Run
1. Clone the repo
   ```
   git clone https://github.com/KalebAsratemedhin/swarm-robotics-in-disaster-scenarios.git
   cd swarm-robotics-in-disaster-scenarios
   ```
3. Make sure you are using ros2-humble and gazebo fortress
  ```
    colcon build
    source install/setup.bash
    ros2 launch my_robot simulation.launch.py
  
  ```
4. 