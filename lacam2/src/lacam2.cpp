#include "../include/lacam2.hpp"

Solution solve(const Instance& ins, std::string& additional_info,
               const int verbose, const Deadline* deadline, std::mt19937* MT,
               const Objective objective, const float restart_rate,
               const PriorityReset wp_reset)
{
  auto planner = Planner(&ins, deadline, MT, verbose, objective, restart_rate,
                         wp_reset);
  return planner.solve(additional_info);
}
