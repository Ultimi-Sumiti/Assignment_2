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
<img width="702" height="578" alt="UR5" src="https://github.com/user-attachments/assets/228daaec-1f29-44d7-a01c-f149bd794621" />


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
* 
## Results:

https://github.com/user-attachments/assets/61e2a5f0-c351-44c9-9161-9c2867fb0b32

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

#Source setup.bash: 
source install/setup.bash
```

### 2. Launch

There are two ways to run the entire pipeline. The first one is to open 5 terminals and then running
in the following order:

```bash
#Terminal 1: run simulation env
ros2 launch ir_launch assignment_2.launch.py

#Terminal 2: run AprilTagNode
ros2 launch src/group_24_assignment_2/launch/launch_AprilTag_node.yml

#Terminal 3: run Gripper
ros2 launch src/group_24_assignment_2/launch/gripper.launch.py

#Terminal 4: run PlannerActionServer
ros2 launch src/group_24_assignment_2/launch/planner_action_server.launch.py

#Terminal 5: run CoordinatorActionClient
ros2 run group_24_assignment_2 coordinator_action_client
```

If you use this method, when you launch `PlannerActionServer`, you will see some errors but you can ignore them.
Everything will work fine.

The second method is to use the global launcher:

```bash
ros2 launch src/group_24_assignment_2/launch/global_launcher.launch.py
```

Note however that, depending on your machine, this could fail. If this is your case, then you should try
to increase the delay of `CoordinatorActionClient` (default to 10 seconds).
To do that you need edit the global launch file like so:

```python
    delayed_coordinator = TimerAction(
        period=10.0, # <-- Increase this
        actions=[
            LogInfo(msg="10 seconds passed. Launching Coordinator..."),
            coordinator_node
        ]
    )
```
