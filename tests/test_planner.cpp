#include <lacam2.hpp>

#include "gtest/gtest.h"

TEST(planner, solve)
{
  const auto scen_filename = "./assets/random-32-32-10-random-1.scen";
  const auto map_filename = "./assets/random-32-32-10.map";
  const auto ins = Instance(scen_filename, map_filename, 3);
  auto additional_info = std::string();

  auto solution = solve(ins, additional_info);
  ASSERT_TRUE(is_feasible_solution(ins, solution));
}

TEST(planner, optimizer)
{
  const auto scen_filename = "./assets/loop.scen";
  const auto map_filename = "./assets/loop.map";
  const auto ins = Instance(scen_filename, map_filename, 3);
  auto additional_info = std::string();

  auto solution_m =
      solve(ins, additional_info, 0, nullptr, nullptr, Objective::OBJ_MAKESPAN);
  ASSERT_TRUE(is_feasible_solution(ins, solution_m));
  ASSERT_TRUE(get_makespan(solution_m) == 10);

  auto solution_l = solve(ins, additional_info, 0, nullptr, nullptr,
                          Objective::OBJ_SUM_OF_LOSS);
  ASSERT_TRUE(is_feasible_solution(ins, solution_l));
  ASSERT_TRUE(get_sum_of_loss(solution_l) == 15);
}

// the memory limit stops the search as the time limit does, keeping the best
// solution found so far
TEST(planner, memory_limit)
{
  const auto ins = Instance("./assets/random-32-32-10-random-1.scen",
                            "./assets/random-32-32-10.map", 50);
  ASSERT_GT(get_memory_mb(), 0);

  // below the memory of the process: stops at the first check, after a
  // first solution (found within MEM_CHECK_PERIOD iterations)
  auto MT = std::mt19937(0);
  const auto deadline = Deadline(60000, 1);
  auto info = std::string();
  const auto sol =
      solve(ins, info, 0, &deadline, &MT, Objective::OBJ_SUM_OF_LOSS);
  ASSERT_TRUE(deadline.memory_exceeded);
  ASSERT_NE(info.find("stopped=memory\n"), std::string::npos);
  ASSERT_NE(info.find("optimal=0\n"), std::string::npos);
  ASSERT_TRUE(is_feasible_solution(ins, sol));

  // a memory limit far away: the time limit stops the search
  auto MT2 = std::mt19937(0);
  const auto deadline2 = Deadline(200, 1 << 20);
  auto info2 = std::string();
  const auto sol2 =
      solve(ins, info2, 0, &deadline2, &MT2, Objective::OBJ_SUM_OF_LOSS);
  ASSERT_FALSE(deadline2.memory_exceeded);
  ASSERT_NE(info2.find("stopped=time\n"), std::string::npos);
  ASSERT_TRUE(is_feasible_solution(ins, sol2));
}
