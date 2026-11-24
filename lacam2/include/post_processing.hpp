/*
 * post processing, e.g., calculating solution quality
 */
#pragma once
#include "dist_table.hpp"
#include "instance.hpp"
#include "utils.hpp"

// time step at which each waypoint is registered along the solution, per
// agent (-1: never); for the final waypoint, the last arrival.
// alive: false if a deadline is missed
std::vector<std::vector<int> > replay_events(const Instance& ins,
                                             const Solution& solution,
                                             bool* alive = nullptr);
bool is_feasible_solution(const Instance& ins, const Solution& solution,
                          const int verbose = 0);
int get_makespan(const Solution& solution);
int get_path_cost(const Solution& solution, uint i);  // single-agent path cost
int get_sum_of_costs(const Solution& solution);
int get_sum_of_loss(const Solution& solution);
// sum of loss aware of waypoints: staying at the final goal is free only
// after completing the sequence
int get_sum_of_loss(const Instance& ins, const Solution& solution);
int get_makespan_lower_bound(const Instance& ins, DistTable& D);
int get_sum_of_costs_lower_bound(const Instance& ins, DistTable& D);
// lower bound on the time each agent completes its sequence: shortest paths
// between its waypoints, ignoring the other agents, plus the waits imposed by
// the temporal constraints (earliest firing time of every event, including
// the precedences between agents). Their sum bounds the sum of costs and the
// sum of loss, their maximum the makespan; without STN, they are the bounds
// above.
std::vector<int> get_completion_lower_bounds(const Instance& ins,
                                             DistTable& D);
void print_stats(const int verbose, const Instance& ins,
                 const Solution& solution, const double comp_time_ms);
void make_log(const Instance& ins, const Solution& solution,
              const std::string& output_name, const double comp_time_ms,
              const std::string& map_name, const int seed,
              const std::string& additional_info,
              const bool log_short = false  // true -> paths not appear
);
