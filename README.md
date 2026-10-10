# Multi-Robot Exploration Test

Hi all! Welcome to the exploration test. Your task is to coordinate a team of **3 robots** so that, together, they explore an unknown environment as fast as possible.

The simulation is the [Multi-Robot Exploration Development Environment](https://github.com/rap-lab-org/Multi-Robot-Exploration-Development-Environment) (an extension of the [CMU Autonomous Exploration Development Environment](https://www.cmu-exploration.com/development-environment)). Each robot already has a lidar, terrain analysis and a **local planner** that drives it to a waypoint while avoiding nearby obstacles. You write the **exploration planner** on top: decide where every robot should go next.

All development must be done in **ROS 2 Humble** with **Gazebo Classic 11** (Ubuntu 22.04). The provided Docker image is the recommended way to get this setup.

## Objective

The test has two parts. Both live in the package **`explorer`** (template in `src/explorer`, C++).

### Part 1: RCH solver

Read [HEHA: Hierarchical Planning for Heterogeneous Multi-Robot Exploration of Unknown Environments](https://arxiv.org/abs/2510.04161) and implement its global routing algorithm, **RCH**.

Given the agents' start positions, a set of targets and a travel cost matrix per agent, the solver assigns every target to exactly one agent and orders each agent's targets. The goal is to minimize the **longest** route (makespan). Routes are open paths: they start at the agent and do not return. A cost of `rch::kInf` means the agent cannot use that edge, which is how heterogeneous robots are modeled.

Implement `rch::solve` in `src/explorer/src/rch.cpp`. The interface is fixed in [`src/explorer/include/explorer/rch.h`](src/explorer/include/explorer/rch.h): do not change it. The provided implementation is only a valid placeholder.

Check your solver with the offline benchmark:

```bash
ros2 run rch_benchmark rch_benchmark --w 1.5 --time 1.0
```

On small instances it compares your makespan with the exact optimum and checks that it is within `w` of it. Every instance must also produce a valid solution within the time limit. Large instances report makespan and runtime only. The final evaluation uses different seeds, `w` values and instance sizes.

### Part 2: Exploration

Use your RCH solver to coordinate the 3 robots during exploration: extract exploration targets from the team map, compute travel costs between robots and targets on the map, solve, and send each robot along its route.

A shared map of the whole team is provided (an `octomap_server` fed by every robot's lidar). Target extraction, global path planning and turning routes into waypoints are up to you.

## Project Structure

```
.
├── docker/                  # Dockerfile and run script
└── src
    ├── environment/         # simulator + local planner (git submodule, do not modify)
    ├── exam_bringup/        # exam launch file, team mapping and evaluator (do not modify)
    ├── rch_benchmark/       # offline RCH benchmark (do not modify)
    └── explorer/            # YOUR package: RCH solver + exploration node
```

## Setup

```bash
git clone --recursive <this repo>
cd <this repo>

# Download the simulation worlds (meshes) into src/environment/src/vehicle_simulator/mesh
./src/environment/src/vehicle_simulator/mesh/download_environments.sh

# Start the container (builds the image on first run); the repo is mounted at /ws
./docker/run.sh

# Inside the container
cd /ws
colcon build --symlink-install
source install/setup.bash
```

## Running

```bash
ros2 launch exam_bringup exam.launch.py world_name:=indoor
```

This starts Gazebo, one autonomy stack per robot (`robot_0`, `robot_1`, `robot_2`), the team map, RViz, the evaluator and your `explorer` (via `explorer/launch/explorer.launch.py`, about 15 s after launch). Pass `start_explorer:=false` to start your explorer yourself, e.g. while debugging.

Useful launch arguments:

| Argument | Default | Meaning |
| --- | --- | --- |
| `world_name` | `indoor` | `indoor`, `garage`, `tunnel` or `forest` |
| `vehicleX_list`, `vehicleY_list` | indoor start poses | Start position of each robot |
| `time_limit` | `300.0` | Evaluation length in seconds |
| `gazebo_gui` | `false` | Show the Gazebo window |

## Interfaces

All positions are in the shared global `map` frame. For each robot `i` in 0, 1, 2:

| Topic | Type | Direction |
| --- | --- | --- |
| `/robot_i/state_estimation` | `nav_msgs/Odometry` | subscribe: robot pose |
| `/robot_i/registered_scan` | `sensor_msgs/PointCloud2` | subscribe: lidar points in `map` |
| `/robot_i/terrain_map` | `sensor_msgs/PointCloud2` | subscribe: local terrain, intensity = height above ground |
| `/robot_i/way_point` | `geometry_msgs/PointStamped` | **publish**: goal for the local planner |

The team map is shared by all robots:

| Topic | Type | Meaning |
| --- | --- | --- |
| `/projected_map` | `nav_msgs/OccupancyGrid` | 2D map (0.5 m cells) projected from obstacles between `occupancy_min_z` and `occupancy_max_z` |
| `/octomap_binary`, `/octomap_full` | `octomap_msgs/Octomap` | 3D occupancy map (0.5 m voxels) |

The mapping parameters for each world are in `src/exam_bringup/config/mapping_<world>.yaml`.

**Note:** the local planner only looks about 4 m around the robot and does not plan around walls. If you send a far waypoint behind an obstacle, the robot will get stuck. Send intermediate waypoints along a global path.

## Evaluation

The evaluator merges the lidar scans of all robots and counts the explored volume (0.5 m voxels, same rule as the CMU environment). Its clock starts when the **first** `way_point` message is published and stops after `time_limit` seconds. It publishes:

* `/exam/explored_volume` and `/exam/elapsed_time` (`std_msgs/Float32`)
* `/exam/explored_voxels` (`sensor_msgs/PointCloud2`, add it in RViz to see coverage)
* `/exam/finished` (`std_msgs/Bool`)

Results are written to `exam_results/<timestamp>/metrics.csv` and `summary.txt`.

Your solution is scored on:

1. **RCH solver**: validity, staying within the `w` bound, and makespan and runtime on hidden benchmark instances.
2. **Exploration**: the team explored volume at the time limit, possibly in environments you have not seen. Robustness matters: robots that get stuck or collide waste time.
3. **Code and write-up**: a short description of how your exploration node uses RCH (targets, costs, replanning).

## Rules

* Do not modify anything under `src/environment`, `src/exam_bringup` or `src/rch_benchmark`, including speed limits and other parameters of the local planner.
* Do not change `rch.h`. The exploration node must use your own RCH implementation for task allocation.
* Only use the topics listed above. Do not read map files (meshes, `.ply`, `.world`) or Gazebo ground truth.
* Robots may share all information with each other, so a central planner is fine.

---

Good luck, and we look forward to seeing your solution!
