#include "explorer/rch.h"

namespace rch
{

Solution solve(const Problem & problem, double /*w*/, double /*time_limit*/)
{
  // TODO: implement RCH.
  // Placeholder: give each target to the first agent that can reach it from
  // its start, in index order. Valid, but far from the bound.
  Solution sol;
  sol.routes.resize(problem.num_agents);
  for (int t = 0; t < problem.num_targets; t++) {
    const int node = problem.num_agents + t;
    for (int a = 0; a < problem.num_agents; a++) {
      if (problem.cost[a][a][node] < kInf) {
        sol.routes[a].push_back(t);
        break;
      }
    }
  }
  return sol;
}

}  // namespace rch
