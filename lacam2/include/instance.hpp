/*
 * instance definition
 */
#pragma once
#include <random>

#include "graph.hpp"
#include "utils.hpp"

// tag to select the waypoint-file constructor
struct WaypointFile {
};

struct TemporalNetwork;  // see stn.hpp

struct Instance {
  const Graph G;
  Config starts;
  Config goals;  // final goals, i.e., goals[i] == waypoints[i].back()
  const uint N;  // number of agents
  // ordered waypoint sequence of each agent, the last one is the final goal
  // for classical MAPF, waypoints[i] = {goals[i]}
  std::vector<Vertices> waypoints;
  // temporal constraints between waypoints, nullptr: none (see stn.hpp)
  std::shared_ptr<const TemporalNetwork> stn;
  // reserved[v]: the only agent allowed on vertex v, -1: anyone;
  // empty: no reserved vertex
  std::vector<int> reserved;

  // for testing
  Instance(const std::string& map_filename,
           const std::vector<uint>& start_indexes,
           const std::vector<uint>& goal_indexes);
  // for testing, with waypoint sequences (vertex indexes, i.e., width * y + x)
  Instance(const std::string& map_filename,
           const std::vector<uint>& start_indexes,
           const std::vector<std::vector<uint> >& waypoint_indexes);
  // for MAPF benchmark
  Instance(const std::string& scen_filename, const std::string& map_filename,
           const uint _N = 1);
  // random instance generation
  Instance(const std::string& map_filename, std::mt19937* MT,
           const uint _N = 1);
  // waypoint file, one agent per line: "sx sy x1 y1 x2 y2 ..."
  Instance(const std::string& map_filename, const std::string& wp_filename,
           const uint _N, WaypointFile);
  // random waypoint instance: m random waypoints per agent,
  // depot=true appends the start location as the final waypoint
  Instance(const std::string& map_filename, std::mt19937* MT, const uint _N,
           const uint m, const bool depot);
  ~Instance() {}

  // number of waypoints of agent i
  inline uint m(uint i) const { return waypoints[i].size(); }
  // all intermediate waypoints of agent i are visited, see advance_K
  inline bool intermediates_done(const std::vector<uint>& K, uint i) const
  {
    return K[i] + 1 == m(i);
  }
  // true if some agent has more than one waypoint
  bool has_waypoints() const;
  // agent i cannot enter v
  inline bool forbidden(uint i, Vertex* v) const
  {
    return !reserved.empty() && reserved[v->id] >= 0 &&
           reserved[v->id] != (int)i;
  }
  // the start of an agent returning there (depot) is reserved for it
  void set_private_depots();

  // simple feasibility check of instance
  bool is_valid(const int verbose = 0) const;
};

// K[i] in {0, ..., m(i)-1}: index of the next waypoint of agent i.
// Only intermediate waypoints are consumed: K[i] == m(i)-1 means "heading to
// the final goal". Reaching the final goal is not recorded in K since the goal
// condition checks locations anyway; this keeps (Q, K) states with identical
// futures merged, and makes m(i) == 1 exactly the classical MAPF search.
// Advance K after agents arrive at C; agent i only consumes its own waypoints.
// 'while' instead of 'if': consecutive identical waypoints are consumed at once.
void advance_K(const Instance& ins, const Config& C, std::vector<uint>& K);
std::vector<uint> initial_K(const Instance& ins);  // K at the start config

// solution: a sequence of configurations
using Solution = std::vector<Config>;
std::ostream& operator<<(std::ostream& os, const Solution& solution);
