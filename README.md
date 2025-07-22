# Exploration Test

Hi, all! Welcome to the test. You are required to work with this framework and develop an exploration algorithm. All development is under `ROS1 Noetic`, please make sure you can work with it.

In brief, **please develop an efficient exploration method under the directory `./src/explorer`, which is able to lead single robot to explore the whole environment.**

Below are some further illustrations.

## Module

The repo is organized as follows.
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

There are several modules, including `local_planner`, `map_builder`, `env`, `odom_manager`. 

**You are required to develop an exploration method under the directory `./src/explorer`, which is able to lead single robot to explore the whole environment.**

## Interfaces

Here listed some interfaces you may need.
```
map_topic: /agv_map
robot_odom_topic: /agv1/odom
target_goal_topic: /agv1/goal
```

## Efficiency Requirement

You are required to write an efficient algorithm such that robot is able  accomplish the exploration task within 3 minutes.