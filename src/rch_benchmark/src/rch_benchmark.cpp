// Offline benchmark for the candidate's RCH solver (explorer/rch.h).
//
// Small instances are compared with the exact optimum (Held-Karp per agent,
// then a subset DP for the min-max split). Every solution is checked for
// validity, the w bound and the time limit. Large instances only report
// makespan and runtime.
//
//   ros2 run rch_benchmark rch_benchmark [--w 1.5] [--time 1.0] [--seed 0]

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <random>
#include <string>
#include <vector>

#include "explorer/rch.h"

using rch::kInf;

namespace
{

constexpr int kAgents = 3;

// Random points in a 100 x 100 square. When heterogeneous, agent a moves with
// its own speed factor and agent 2 cannot reach about a quarter of the targets
// (every target stays reachable by agents 0 and 1).
rch::Problem makeInstance(int num_targets, bool heterogeneous, std::mt19937 & rng)
{
  std::uniform_real_distribution<double> coord(0.0, 100.0), unit(0.0, 1.0);
  const int n = kAgents + num_targets;
  std::vector<double> x(n), y(n);
  for (int i = 0; i < n; i++) { x[i] = coord(rng); y[i] = coord(rng); }

  std::vector<double> speed(kAgents, 1.0);
  std::vector<bool> blocked(n, false);
  if (heterogeneous) {
    speed = {1.0, 0.7, 1.5};
    for (int i = kAgents; i < n; i++) blocked[i] = unit(rng) < 0.25;
  }

  rch::Problem p;
  p.num_agents = kAgents;
  p.num_targets = num_targets;
  p.cost.assign(kAgents, std::vector<std::vector<double>>(n, std::vector<double>(n, 0.0)));
  for (int a = 0; a < kAgents; a++) {
    for (int i = 0; i < n; i++) {
      for (int j = 0; j < n; j++) {
        const bool no = a == 2 && (blocked[i] || blocked[j]);
        p.cost[a][i][j] = no ? kInf : std::hypot(x[i] - x[j], y[i] - y[j]) / speed[a];
      }
    }
  }
  return p;
}

// Returns the makespan, or a negative value with `error` set if invalid.
double evaluate(const rch::Problem & p, const rch::Solution & s, std::string & error)
{
  if (static_cast<int>(s.routes.size()) != p.num_agents) {
    error = "routes.size() != num_agents";
    return -1;
  }
  std::vector<int> seen(p.num_targets, 0);
  double makespan = 0;
  for (int a = 0; a < p.num_agents; a++) {
    int prev = a;
    double c = 0;
    for (int t : s.routes[a]) {
      if (t < 0 || t >= p.num_targets) { error = "target index out of range"; return -1; }
      seen[t]++;
      c += p.cost[a][prev][p.num_agents + t];
      prev = p.num_agents + t;
    }
    if (!std::isfinite(c)) { error = "agent " + std::to_string(a) + " uses an untraversable edge"; return -1; }
    makespan = std::max(makespan, c);
  }
  for (int t = 0; t < p.num_targets; t++) {
    if (seen[t] != 1) {
      error = "target " + std::to_string(t) + " visited " + std::to_string(seen[t]) + " times";
      return -1;
    }
  }
  return makespan;
}

double optimalMakespan(const rch::Problem & p)
{
  const int m = p.num_targets, full = 1 << m;

  // best[a][S]: cheapest open path of agent a from its start through set S.
  std::vector<std::vector<double>> best(p.num_agents, std::vector<double>(full, kInf));
  for (int a = 0; a < p.num_agents; a++) {
    const auto & c = p.cost[a];
    std::vector<double> dp(static_cast<size_t>(full) * m, kInf);
    for (int j = 0; j < m; j++) dp[(1 << j) * m + j] = c[a][p.num_agents + j];
    for (int S = 1; S < full; S++) {
      for (int j = 0; j < m; j++) {
        const double v = dp[S * m + j];
        if (!(S >> j & 1) || v == kInf) continue;
        best[a][S] = std::min(best[a][S], v);
        for (int k = 0; k < m; k++) {
          if (S >> k & 1) continue;
          double & nxt = dp[(S | 1 << k) * m + k];
          nxt = std::min(nxt, v + c[p.num_agents + j][p.num_agents + k]);
        }
      }
    }
    best[a][0] = 0;
  }

  // f[S]: optimal makespan when the agents processed so far cover set S.
  std::vector<double> f = best[0];
  for (int a = 1; a < p.num_agents; a++) {
    std::vector<double> g(full, kInf);
    for (int S = 0; S < full; S++) {
      for (int T = S;; T = (T - 1) & S) {
        g[S] = std::min(g[S], std::max(f[S ^ T], best[a][T]));
        if (T == 0) break;
      }
    }
    f.swap(g);
  }
  return f[full - 1];
}

}  // namespace

int main(int argc, char ** argv)
{
  double w = 1.5, time_limit = 1.0;
  unsigned seed = 0;
  for (int i = 1; i + 1 < argc; i += 2) {
    if (!std::strcmp(argv[i], "--w")) w = std::atof(argv[i + 1]);
    else if (!std::strcmp(argv[i], "--time")) time_limit = std::atof(argv[i + 1]);
    else if (!std::strcmp(argv[i], "--seed")) seed = std::atoi(argv[i + 1]);
  }
  std::printf("RCH benchmark: %d agents, w = %.2f, time limit = %.2f s, seed = %u\n\n",
              kAgents, w, time_limit, seed);
  std::printf("%-6s %-8s %4s %12s %12s %8s %10s  %s\n",
              "suite", "type", "M", "makespan", "optimal", "ratio", "time [s]", "result");

  std::mt19937 rng(seed);
  int failed = 0, total = 0;
  const int kPerSize = 3;

  auto run = [&](int m, bool het, bool exact) {
    const rch::Problem p = makeInstance(m, het, rng);
    const auto t0 = std::chrono::steady_clock::now();
    const rch::Solution s = rch::solve(p, w, time_limit);
    const double secs = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();

    std::string error;
    const double ms = evaluate(p, s, error);
    const double opt = exact ? optimalMakespan(p) : NAN;
    if (ms >= 0 && exact && ms > w * opt + 1e-6) error = "exceeds w * optimal";
    if (secs > time_limit + 0.05) error = error.empty() ? "time limit exceeded" : error;

    total++;
    if (!error.empty()) failed++;
    std::printf("%-6s %-8s %4d %12.2f %12.2f %8.3f %10.3f  %s\n",
                exact ? "small" : "large", het ? "hetero" : "homo", m, ms, opt,
                exact ? ms / opt : NAN, secs, error.empty() ? "ok" : ("FAIL: " + error).c_str());
  };

  for (bool het : {false, true}) {
    for (int m = 4; m <= 10; m += 2) {
      for (int k = 0; k < kPerSize; k++) run(m, het, true);
    }
  }
  for (bool het : {false, true}) {
    for (int m : {30, 60}) {
      for (int k = 0; k < kPerSize; k++) run(m, het, false);
    }
  }

  std::printf("\n%d / %d instances passed\n", total - failed, total);
  return failed == 0 ? 0 : 1;
}
