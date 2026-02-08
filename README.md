# Warehouse Automation using ROS 2

This repository contains a warehouse automation project built using ROS 2, combining a UR5 robotic arm and an Autonomous Ground Vehicle (AGV) to demonstrate end-to-end automation tasks commonly found in modern warehouse environments.

The project focuses on autonomous navigation using SLAM, coordinated robot motion, and task execution in a simulated warehouse setup.

---

## Project Overview

The system represents a warehouse scenario where:

* An AGV autonomously navigates the warehouse using SLAM and the ROS 2 Navigation Stack (Nav2).
* A UR5 robotic arm performs manipulation tasks such as pick-and-place operations.
* Both systems operate within a unified ROS 2 ecosystem, enabling modular, scalable, and realistic automation workflows.

This setup reflects real-world warehouse automation pipelines involving mobile robots integrated with industrial manipulators.

---


## System Components

### Autonomous Ground Vehicle (AGV)

* ROS 2-based mobile robot platform
* SLAM-based map generation and localization
* Navigation implemented using Nav2 (global planning, local planning, and obstacle avoidance)
* Goal-based autonomous navigation

### UR5 Robotic Arm

* 6-DOF industrial robotic manipulator
* Integrated using ROS 2 control and motion planning frameworks
* Executes predefined manipulation tasks
* Designed to interact with objects transported by the AGV

---


## Tech Stack

* ROS 2 (Humble or later)
* nav2
* slam toolbox
* moveIt2
* Gazebo 
* rViz

---

## How to Run

1. Clone the repository:

   ```bash
   git clone https://github.com/Rishitha2628/logistic-bot.git
   cd logisticBot
   ```

2. Build the ROS 2 workspace:

   ```bash
   colcon build
   source install/setup.bash
   ```


---

## Demo Video

Project demonstration video:

[Watch the demo video](https://youtu.be/Ah8l9qyM8Ow)

---

## Future Work

* Real-time coordination between AGV and UR5
* Multi-AGV support
* Vision-based perception and object detection
* Task scheduling and fleet management

---


## License

This project is licensed under the MIT License. See the LICENSE file for details.


