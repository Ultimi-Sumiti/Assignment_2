# Assignment_2
<p align="center">
  <img width="350" height="850" alt="icon3" src="https://github.com/user-attachments/assets/eb4f645d-238b-4ed0-827c-fcbbdbd245d5" />
</p>

## 📖 Table of Contents

* [Description](#description)
* [Goal](#goal)
* [Pipeline Components](#pipeline-components)
* [Flow and ROS2 interactions](#flow-and-ros2-interactions)
    * [1. Initialization and Perception phase](#1-initialization-and-perception-phase)
    * [2. Planning and Execution phase](#2-planning-and-execution-phase)
* [Results:](#results)    
* [How to launch](#how-to-launch)
    * [1. Prerequisites (If not done)](#1-prerequisites-if-not-done)

## Description:

This repository contains the solution for "Assignment 2: The UR5 + MoveIt" for the course of Intelligent Robotics. The objective of this assignment is to develop a complete ROS 2 pipeline for the **UR5 robotic arm** to perform a complex manipulation task: detecting objects via camera, planning trajectories using **MoveIt**, and executing a pick-and-place sequence to swap two cubes in a simulated Gazebo environment.

## Goal:

The core mission for the **UR5** is to:
1.  **Detect** the position of two cubes (Red Cube/Tag 10 and Blue Cube/Tag 1) using an external camera and the TF tree.
2.  **Pick** the first cube (Tag 10) from its starting table.
3.  **Place** it on the opposing table (where Tag 1 is located).
4.  **Pick** the second cube (Tag 1).
5.  **Place** it on the first table (completing the swap).


The robot spawns in a simulation environment close to two tables and an external camera:
<p align="center" >
<img width="419" height="496" alt="final_position" src="https://github.com/user-attachments/assets/db24c7d0-7621-4ddb-802e-b1419a106c0e" />
</p>

## Pipeline Components:

The final pipeline, developed as a modular ROS 2 package, includes the following components/nodes:

* **Coordinator action client:** This is the core reasoning node, it works as if he were the director and coordinates all the various actions that need to be carried out by communicating what needs to be done at that moment with the `planner` and the `gripper`.

* **Planner action server:** This node is exclusively dedicated to motion planning and execution. It exploits an **Action Interface** used by the Coordinator to request arm movements. It supports two specific planning modes: "free_cartesian" (standard point-to-point planning) and "path_cartesian" (linear trajectories for precise approach/retreat phases).

* **Gripper:** This node is responsible solely for the actuation of the end-effector (opening and closing). It communicates with the Coordinator using **ROS 2 Topics**: it subscribes to a command topic where the Coordinator specifies the target opening degrees, and it publishes the current state of the gripper on a separate status topic to specify when its job is completed.

## Flow and ROS2 interactions:

### 1. Initialization phase
Initially, the `planner action server` and the `gripper` node are launched. They independently initialize their interfaces with the **MoveIt** framework (MoveGroupInterface, PlanningScene, etc.) and establish the necessary communication channels (Action Servers and Topics). Once initialized, they enter a standby state, waiting for external commands which will be given by the `coordinator`.

### 2. Coordination and Execution phase
The operational pipeline is triggered effectively once the `coordinator action client` is launched. This node acts as the central brain, containing the specific logic of the pick-and-place task. It orchestrates the entire workflow by dynamically calling the other nodes according to the chosen state machine:
* **Sequence Management**: The coordinator determines the step-by-step pipeline (Approach Cube 1 -> Open Gripper -> Descend -> Close Gripper -> Lift -> Move to Table 2 -> ...).
* **Actuation gripper Calls**: It commands the `gripper` to open or close at precise moments via ROS 2 topics.
* **Motion planning Requests**: It sends `Plan` goals to the `planner` to execute specific trajectories (e.g., requesting a "free_cartesian" path to approach the table or a "path_cartesian" for the vertical grasp) based on the target poses retrieved from the perception system.
## Results:


## How to launch

This solution is designed to run within the pre-configured workspace from the assignment setup.

### 1. Prerequisites (If not done)

If you haven't already, please follow the initial setup steps to initialize your dedicated workspace and clone the necessary packages (`ir_2526`):

```bash
#Clone the repository:
git clone https://github.com/Ultimi-Sumiti/Assignment_2

#Position in the correct directory/workspace:
cd Assignment_2
cd ws_24_assignments

#Build the package:
colcon build

#Launch setup.bash: 
source install/setup.bash

#Launch the launch file:
ros2 launch group_24_assignment_2 launch.py