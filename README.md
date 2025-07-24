# Exploration Test

Hi all! Welcome to the exploration test. Your task is to develop an exploration algorithm using the provided framework.

All development must be done in **ROS1 Noetic**. Please ensure that your environment is properly set up for ROS Noetic before proceeding.

## Objective

You are required to develop an efficient exploration package named **`explorer`** that enables a single robot to fully explore an unknown environment.


## Basic Knowledge of ROS

You need to develop a ROS1 package, which is a self-contained unit in the Robot Operating System (ROS) that includes source code, configuration, and dependencies. In ROS, nodes, which are basic units to perform specific function, communicate by publishing and subscribing to topics, which are named channels carrying messages. A node advertises a topic when it sends messages, and subscribes to a topic to receive messages from other nodes. For example, this exploration package subscribes to the robot’s odometry (/agv1/odom) and map (/agv_map) topics to perceive the environment, and publishes target goals to /agv1/goal to command the robot’s movement. This pub-sub mechanism enables modular and flexible robot control.

## Project Structure

The repository is organized as follows. Each directory under `src` is a package in ROS.

```
.
├── README.md
└── src
    ├── local_planner
    ├── map_builder
    ├── odom_manager
    ├── path_finder
    ├── pcd_publisher
    ├── simulator
    └── trajectory_tracker
```

You will implement your exploration algorithm as a package under

```
src/explorer/ # you need to make this directory by yourself.
```

## Interfaces

The following ROS topics are available and may be useful for your implementation:

* **Map topic**: `/agv_map`
* **Odometry topic**: `/agv1/odom`
* **Goal publishing topic**: `/agv1/goal`

Make sure to correctly subscribe to and publish on these topics to interact with the system.

## Performance Requirement

Your algorithm must be **efficient enough to complete the full environment exploration within 3 minutes**.

---

Good luck, and we look forward to seeing your solution!
