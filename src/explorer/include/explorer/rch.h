// RCH global routing solver -- implemented by the candidate.
//
// Reference: Yang et al., "HEHA: Hierarchical Planning for Heterogeneous
// Multi-Robot Exploration of Unknown Environments", arXiv:2510.04161.
//
// Do not change this interface: the benchmark in src/rch_benchmark and the
// final evaluation link against it.

#pragma once

#include <limits>
#include <vector>

namespace rch
{

constexpr double kInf = std::numeric_limits<double>::infinity();

// Nodes 0 .. num_agents-1 are the agents' start positions,
// nodes num_agents .. num_agents+num_targets-1 are the targets.
struct Problem
{
  int num_agents = 0;
  int num_targets = 0;
  // cost[a][i][j]: cost for agent a to travel from node i to node j.
  // kInf means agent a cannot traverse that edge (heterogeneous robots).
  std::vector<std::vector<std::vector<double>>> cost;
};

struct Solution
{
  // routes[a]: targets visited by agent a, in order, as target indices
  // 0 .. num_targets-1 (not node indices). Routes are open paths that start
  // at the agent's start node and do not return.
  std::vector<std::vector<int>> routes;
};

// Assign every target to exactly one agent and order each agent's targets so
// that the longest route cost (makespan) is minimized.
//
// w:          suboptimality bound (>= 1); the returned makespan must be at
//             most w times the optimal makespan.
// time_limit: wall-clock budget in seconds.
Solution solve(const Problem & problem, double w, double time_limit);

}  // namespace rch
