/*
 * temporal constraints between waypoints: Simple Temporal Network (STN)
 *
 * An event (i, k) is the instant agent i registers its k-th intermediate
 * waypoint; event 0 is the origin, at time 0. The final waypoint has no event.
 *
 * Supported: lower bounds between events (precedences, delays) and absolute
 * windows (from the origin, e.g., release times and deadlines). Upper bounds
 * between two events are rejected: with lower bounds only, firing an event at
 * the first possible instant is dominant, so firing needs no search.
 *
 * File format ('#' starts a comment):
 *   timepoints T
 *   <agent> <k>           T lines, timepoints 1..T (0 is the origin);
 *                         agent -1: free timepoint (e.g., an operation),
 *                         projected away
 *   constraints M
 *   <a> <b> <lb> <ub>     lb <= t_b - t_a <= ub, '-' for unbounded
 */
#pragma once

#include <climits>
#include <memory>

#include "instance.hpp"
#include "utils.hpp"

struct DistTable;

struct TemporalNetwork {
  static constexpr int INF = INT_MAX / 4;
  uint E;                                  // number of events, origin included
  std::vector<std::vector<int> > event_of;  // [i][k] -> event, -1: none
  std::vector<uint> agent;                 // event -> agent (origin: N)
  std::vector<uint> index;                 // event -> waypoint index k
  // minimal network, D[a][b] = max of t_b - t_a (INF: unbounded)
  std::vector<std::vector<int> > D;
  // preds[e]: events of other agents that must fire no later than e
  std::vector<std::vector<uint> > preds;
  bool has_deadline;  // some event has a finite upper bound
  bool valid;
  std::string error;

  TemporalNetwork(std::istream& is, const Instance& ins);
  TemporalNetwork(const std::string& filename, const Instance& ins);

 private:
  void load(std::istream& is, const Instance& ins);
  bool fail(const std::string& msg);
};

// time window of an unexecuted event, relative to the current time step
struct Window {
  uint e;
  int lo;  // fires in lo steps at the earliest, > 0 when stored
  int hi;  // must fire within hi steps, INF: no deadline
  bool operator==(const Window& w) const
  {
    return e == w.e && lo == w.lo && hi == w.hi;
  }
};
// windows of unexecuted events with an active bound (lo > 0 or hi < INF),
// sorted by event; empty without STN
using Windows = std::vector<Window>;

uint hash_windows(const Windows& W, uint hash);

// waypoint progress and windows, i.e., (K, W) of the state (Q, K, W).
// advance_state: one time step elapsed and agents arrived at C; shift the
// windows and fire events (agent on its current waypoint, predecessors fired,
// window open). Returns false if a deadline is missed (dead state).
// Without STN, it is advance_K.
bool advance_state(const Instance& ins, const Config& C, std::vector<uint>& K,
                   Windows& W, const bool shift = true);
// state at the start configuration, false if a deadline is missed
bool initial_state(const Instance& ins, std::vector<uint>& K, Windows& W);

// earliest number of steps until the current event of agent i can fire:
// 0 without event, INF if some predecessor has not fired yet
int event_release(const Instance& ins, const std::vector<uint>& K,
                  const Windows& W, uint i);

// lower bound on the time to complete the sequence of agent i from v,
// including the waits for the windows of its own events; INF if a deadline
// cannot be met
uint time_to_go(const Instance& ins, DistTable& D, const std::vector<uint>& K,
                const Windows& W, uint i, Vertex* v);

// latest time at which agent i can register its k-th waypoint, dl[i][k], so
// that every agent completes its sequence by 'target' (dl[i][m_i - 1] =
// target): backward propagation of target through the travel times between
// consecutive waypoints and the STN (absolute deadlines included). Without
// STN, dl[i][k] = target - rem[i][k]. Used for the slack-based priorities.
std::vector<std::vector<int> > waypoint_deadlines(const Instance& ins,
                                                  DistTable& D,
                                                  const int target);

// drop the lower bounds of W implied by the travel times (they cannot bind):
// states differing only by them have the same future, merging them keeps the
// state space small and lets EXPLORED detect cycles
void prune_windows(const Instance& ins, DistTable& D, const Config& C,
                   const std::vector<uint>& K, Windows& W);
