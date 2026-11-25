#include <lacam2.hpp>

#include "gtest/gtest.h"

// empty-8-8: vertex index = 8 * y + x
static const auto EMPTY = "./assets/empty-8-8.map";
static uint xy(uint x, uint y) { return 8 * y + x; }

// depot D=(0,0) -> X=(0,3) -> D
// the final configuration equals the initial one: with EXPLORED keyed only
// by configurations it would be discarded and the search would fail
TEST(Waypoints, depot_return)
{
  const auto ins = Instance(EMPTY, "./assets/waypoints-depot.wp", 1,
                            WaypointFile());
  ASSERT_TRUE(ins.is_valid());
  ASSERT_EQ(ins.m(0), 2);
  ASSERT_EQ(ins.starts[0]->index, xy(0, 0));
  ASSERT_EQ(ins.waypoints[0][0]->index, xy(0, 3));
  ASSERT_EQ(ins.goals[0]->index, xy(0, 0));

  auto info = std::string();
  const auto sol = solve(ins, info);
  ASSERT_FALSE(sol.empty());
  ASSERT_TRUE(is_feasible_solution(ins, sol));
  ASSERT_EQ(get_makespan(sol), 6);
  ASSERT_EQ(get_sum_of_loss(ins, sol), 6);
}

TEST(Waypoints, advance_K)
{
  // agent 0: X=(3,0) -> G0=(7,0), agent 1: Y=(0,3) -> G1=(0,7)
  const auto ins = Instance(EMPTY, {xy(0, 0), xy(1, 1)},
                            std::vector<std::vector<uint> >(
                                {{xy(3, 0), xy(7, 0)}, {xy(0, 3), xy(0, 7)}}));
  ASSERT_TRUE(ins.is_valid());
  auto K = initial_K(ins);
  ASSERT_EQ(K, std::vector<uint>({0, 0}));

  // agent 0 on the waypoint of agent 1: nobody progresses
  advance_K(ins, Config({ins.G.U[xy(0, 3)], ins.G.U[xy(1, 1)]}), K);
  ASSERT_EQ(K, std::vector<uint>({0, 0}));

  // agent 1 on its own waypoint
  advance_K(ins, Config({ins.G.U[xy(0, 0)], ins.G.U[xy(0, 3)]}), K);
  ASSERT_EQ(K, std::vector<uint>({0, 1}));

  // reaching the final goal is not recorded, K stays at m - 1
  advance_K(ins, Config({ins.G.U[xy(0, 0)], ins.G.U[xy(0, 7)]}), K);
  ASSERT_EQ(K, std::vector<uint>({0, 1}));

  // K is monotone
  advance_K(ins, Config({ins.G.U[xy(0, 0)], ins.G.U[xy(1, 1)]}), K);
  ASSERT_EQ(K, std::vector<uint>({0, 1}));
}

TEST(Waypoints, consecutive_identical_waypoints)
{
  // X, X, Y: both X are consumed in the same step ('while', not 'if')
  const auto X = xy(0, 3), Y = xy(7, 0);
  const auto ins = Instance(EMPTY, {xy(0, 0)},
                            std::vector<std::vector<uint> >({{X, X, Y}}));
  auto K = initial_K(ins);
  advance_K(ins, Config({ins.G.U[X]}), K);
  ASSERT_EQ(K[0], 2);

  // a waypoint at the start location is consumed at t=0
  const auto ins2 = Instance(EMPTY, {X},
                             std::vector<std::vector<uint> >({{X, X, Y}}));
  ASSERT_EQ(initial_K(ins2)[0], 2);

  // optimal makespan: 3 + dist((0,3), (7,0)) = 3 + 10
  auto info = std::string();
  const auto sol =
      solve(ins, info, 0, nullptr, nullptr, Objective::OBJ_MAKESPAN);
  ASSERT_TRUE(is_feasible_solution(ins, sol));
  ASSERT_EQ(get_makespan(sol), 13);
}

TEST(Waypoints, feasibility_checks_order)
{
  // (0,0) -> (1,0) -> (0,1) -> (1,1)
  const auto ins =
      Instance(EMPTY, {xy(0, 0)},
               std::vector<std::vector<uint> >({{xy(1, 0), xy(0, 1), xy(1, 1)}}));
  auto path = [&](std::vector<uint> idx) {
    auto sol = Solution();
    for (auto k : idx) sol.push_back(Config({ins.G.U[k]}));
    return sol;
  };
  // correct order
  ASSERT_TRUE(is_feasible_solution(
      ins, path({xy(0, 0), xy(1, 0), xy(0, 0), xy(0, 1), xy(1, 1)})));
  // wrong order: (0,1) before (1,0)
  ASSERT_FALSE(is_feasible_solution(
      ins, path({xy(0, 0), xy(0, 1), xy(0, 0), xy(1, 0), xy(1, 1)})));
  // waypoint skipped
  ASSERT_FALSE(is_feasible_solution(ins, path({xy(0, 0), xy(0, 1), xy(1, 1)})));
}

TEST(Waypoints, sum_of_loss)
{
  // waiting at the final goal before visiting the waypoint is not free
  const auto ins = Instance(EMPTY, {xy(0, 0)},
                            std::vector<std::vector<uint> >({{xy(1, 0), xy(0, 0)}}));
  auto sol = Solution();
  for (auto k : {xy(0, 0), xy(0, 0), xy(1, 0), xy(0, 0)})
    sol.push_back(Config({ins.G.U[k]}));
  ASSERT_TRUE(is_feasible_solution(ins, sol));
  ASSERT_EQ(get_sum_of_loss(ins, sol), 3);
  ASSERT_EQ(get_sum_of_loss(sol), 2);  // classical definition
}

// one waypoint per agent is exactly classical MAPF
TEST(Waypoints, single_waypoint_equals_classical)
{
  const auto scen = "./assets/random-32-32-10-random-1.scen";
  const auto map = "./assets/random-32-32-10.map";
  const auto ins = Instance(scen, map, 30);
  auto starts = std::vector<uint>();
  auto seqs = std::vector<std::vector<uint> >();
  for (uint i = 0; i < ins.N; ++i) {
    starts.push_back(ins.starts[i]->index);
    seqs.push_back({ins.goals[i]->index});
  }
  const auto ins_wp = Instance(map, starts, seqs);
  // same search, i.e., same solution and same number of search nodes
  // (anytime objectives are not compared since they depend on the time limit)
  auto info1 = std::string(), info2 = std::string();
  auto MT1 = std::mt19937(0), MT2 = std::mt19937(0);
  const auto s1 = solve(ins, info1, 0, nullptr, &MT1);
  const auto s2 = solve(ins_wp, info2, 0, nullptr, &MT2);
  ASSERT_FALSE(s1.empty());
  ASSERT_EQ(info1, info2);
  ASSERT_EQ(s1.size(), s2.size());
  for (size_t t = 0; t < s1.size(); ++t)
    ASSERT_TRUE(is_same_config(s1[t], s2[t]));
}

TEST(Waypoints, random_instances)
{
  const auto map = "./assets/random-32-32-10.map";
  for (uint seed = 0; seed < 3; ++seed) {
    for (uint N : {10, 30}) {
      for (uint m : {2, 4}) {
        for (bool depot : {false, true}) {
          for (auto [reset, prio] :
               {std::pair(WP_RESET_FINAL, PRIO_DIST),
                std::pair(WP_RESET_EACH, PRIO_DIST),
                std::pair(WP_RESET_EACH, PRIO_SLACK)}) {  // slack: no reset
            auto MT = std::mt19937(seed);
            const auto ins = Instance(map, &MT, N, m, depot);
            ASSERT_TRUE(ins.is_valid());
            for (uint i = 0; i < N; ++i) {
              ASSERT_EQ(ins.m(i), depot ? m + 1 : m);
              if (depot) {
                ASSERT_EQ(ins.goals[i], ins.starts[i]);
              }
            }
            auto info = std::string();
            const auto deadline = Deadline(5000);
            const auto sol = solve(ins, info, 0, &deadline, &MT, OBJ_NONE,
                                   0.001, reset, prio);
            ASSERT_FALSE(sol.empty())
                << "seed=" << seed << " N=" << N << " m=" << m
                << " depot=" << depot << " reset=" << reset
                << " priority=" << prio;
            ASSERT_TRUE(is_feasible_solution(ins, sol));
          }
        }
      }
    }
  }
}

TEST(Waypoints, lacam_star)
{
  const auto map = "./assets/random-32-32-10.map";
  auto MT = std::mt19937(0);
  const auto ins = Instance(map, &MT, 5, 3, true);
  auto D = DistTable(ins);
  auto info = std::string();

  const auto sol = solve(ins, info);
  ASSERT_TRUE(is_feasible_solution(ins, sol));

  const auto deadline_m = Deadline(2000);
  const auto sol_m =
      solve(ins, info, 0, &deadline_m, nullptr, Objective::OBJ_MAKESPAN);
  ASSERT_TRUE(is_feasible_solution(ins, sol_m));
  ASSERT_LE(get_makespan(sol_m), get_makespan(sol));
  ASSERT_GE(get_makespan(sol_m), get_makespan_lower_bound(ins, D));

  const auto deadline_l = Deadline(2000);
  const auto sol_l =
      solve(ins, info, 0, &deadline_l, nullptr, Objective::OBJ_SUM_OF_LOSS);
  ASSERT_TRUE(is_feasible_solution(ins, sol_l));
  ASSERT_LE(get_sum_of_loss(ins, sol_l), get_sum_of_loss(ins, sol));
  ASSERT_GE(get_sum_of_loss(ins, sol_l), get_sum_of_costs_lower_bound(ins, D));
}

TEST(Waypoints, invalid_files)
{
  ASSERT_FALSE(
      Instance(EMPTY, "./assets/not-found.wp", 1, WaypointFile()).is_valid());

  const auto filename = testing::TempDir() + "lacam2-invalid.wp";
  auto check = [&](const std::string& content) {
    std::ofstream(filename) << content;
    return Instance(EMPTY, filename, 1, WaypointFile()).is_valid();
  };
  ASSERT_TRUE(check("# comment\n\n0 0 1 1\n"));
  ASSERT_FALSE(check("0 0\n"));          // no waypoint
  ASSERT_FALSE(check("0 0 1\n"));        // odd number of coordinates
  ASSERT_FALSE(check("0 0 9 9\n"));      // out of map
  ASSERT_FALSE(check("0 0 1 a\n"));      // not a number
  std::remove(filename.c_str());

  // final goals must be distinct
  ASSERT_FALSE(Instance(EMPTY, {xy(0, 0), xy(1, 1)},
                        std::vector<std::vector<uint> >({{xy(3, 3)}, {xy(3, 3)}}))
                   .is_valid());
}
