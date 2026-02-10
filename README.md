# Assignment 2 – Research Track I

**Course:** Research Track I  
**Instructor:** Carmine Tommaso Recchiuto  
**Student Name:** Richard Albert K M  
**Student Number:** 8525970  

---

## Overview

This repository contains the solution for **Assignment 2** of the Research Track I course.
The assignment extends the concepts of Assignment 1 by introducing sensor-based safety,
custom ROS interfaces, services, and runtime interaction with the robot.

The system is implemented in **ROS 2 (Humble)** and tested in a Gazebo simulation
environment equipped with a laser scanner.

---

## Assignment Requirements

The implemented system satisfies the following requirements:

- Allow the user to drive the robot by setting **linear and angular velocity**
- Continuously monitor the environment using **laser scanner data**
- If the robot becomes **too close to an obstacle**:
  - Move the robot backward to return to a safe area
  - Stop automatically once a safe distance is restored
- Allow the user to **change the safety threshold** using a **ROS service**
- Publish a **custom message** containing:
  - Distance of the closest obstacle
  - Direction of the obstacle (left, front, right)
  - Current threshold value
- Provide a **ROS service** that returns the **average linear and angular velocity**
  of the **most recent 5 user commands**

---

## Package Structure

The project is organized into two ROS 2 packages:

### 1. Interface Package (ament_cmake)

`assignment2_custom_msg`

assignment2_custom_msg/
├── msg/
│ └── ObstacleInfo.msg
├── srv/
│ ├── SetThreshold.srv
│ └── GetAverageVelocity.srv
├── CMakeLists.txt
└── package.xml


This package contains all custom message and service definitions.

---

### 2. Logic Package (ament_python)

`assignment2_rt`

assignment2_rt/
├── assignment2_rt/
│ ├── safety_node.py
│ └── ui_node.py
├── package.xml
├── setup.py
└── README.md


This package contains the implementation of the system logic.

---

## Nodes Description

### Safety Node

The **Safety Node** is responsible for monitoring the environment and enforcing safety
constraints using laser scanner data.

Main features:
- Subscribes to `/scan` and `/odom`
- Computes the closest obstacle distance
- Determines obstacle direction (left / front / right)
- Publishes obstacle information using a custom message
- Moves the robot backward when the distance is below the threshold
- Stops the robot once the environment is safe again
- Allows the threshold to be changed via a ROS service

**Published Topics:**
- `/cmd_vel`
- `/closest_obstacle`

**Services:**
- `/set_threshold`

---

### UI Node

The **UI Node** provides a simple terminal-based interface for user interaction.

Main features:
- Allows the user to input linear and angular velocities
- Publishes velocity commands to the robot
- Automatically stops the robot after 1 second
- Stores the last 5 velocity commands
- Provides a ROS service to compute the average velocity

**Published Topics:**
- `/cmd_vel`

**Services:**
- `/get_average_velocity`

---

## Custom Interfaces

### Custom Message

**ObstacleInfo.msg**

float32 distance
string direction
float32 threshold


This message is published continuously by the Safety Node to report obstacle information.

---

### Custom Services

**SetThreshold.srv**

float32 threshold

bool success


Used to update the safety distance threshold at runtime.

---

**GetAverageVelocity.srv**

float32 avg_linear
float32 avg_angular


Returns the average linear and angular velocity of the last 5 user commands.

---

## How to Build the Workspace

From the workspace root:

```bash
colcon build
source install/setup.bash

How to Run the Assignment
1. Launch the simulation environment

ros2 launch bme_gazebo_sensors world.launch.py

2. Run the Safety Node

ros2 run assignment2_rt safety_node

3. Run the UI Node

ros2 run assignment2_rt ui_node

Runtime Interaction
Change the safety threshold

ros2 service call /set_threshold assignment2_custom_msg/srv/SetThreshold "{threshold: 1.2}"

Get the average velocity of the last 5 commands

ros2 service call /get_average_velocity assignment2_custom_msg/srv/GetAverageVelocity

Monitor obstacle information

ros2 topic echo /closest_obstacle

Design Notes

    A recovery state is used to prevent infinite backward motion.

    The robot moves backward only while the safety condition is violated.

    Once the distance becomes safe again, a zero velocity command is sent.

    User input is handled synchronously using terminal input.

    spin_once() is used to allow service callbacks while waiting for input.
