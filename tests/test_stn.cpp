#include <lacam2.hpp>

#include "gtest/gtest.h"

// empty-8-8: vertex index = 8 * y + x
static const auto EMPTY = "./assets/empty-8-8.map";
static uint xy(uint x, uint y) { return 8 * y + x; }

static std::shared_ptr<TemporalNetwork> make_stn(const Instance& ins,
                                                 const std::string& text)
{
  std::istringstream is(text);
  return std::make_shared<TemporalNetwork>(is, ins);
}

static Instance make_ins(const std::vector<uint>& starts,
                         const std::vector<std::vector<uint> >& seqs)
{
  return Instance(EMPTY, starts, seqs);
}

static std::string get_info(const std::string& info, const std::string& key)
{
  std::istringstream is(info);
  std::string line;
  while (std::getline(is, line))
    if (line.rfind(key + "=", 0) == 0) return line.substr(key.size() + 1);
  return "";
}

// drop then pickup on the same cell, at least 5 steps apart (processing):
// the robot has to wait, i.e., revisit the same (Q, K)
TEST(STN, wait_on_machine)
{
  const auto X = xy(3, 0);
  auto ins = make_ins({xy(0, 0)}, {{X, X, xy(0, 0)}});
  ins.stn = make_stn(ins, "timepoints 2\n0 0\n0 1\nconstraints 1\n1 2 5 -\n");
  ASSERT_TRUE(ins.stn->valid) << ins.stn->error;

  auto info = std::string();
  const auto sol = solve(ins, info);
  ASSERT_TRUE(is_feasible_solution(ins, sol, 1));
  const auto times = replay_events(ins, sol);
  ASSERT_EQ(times[0][0], 3);
  ASSERT_EQ(times[0][1], 8);
  ASSERT_EQ(get_makespan(sol), 11);

  // optimal, with the lower bound from the windows
  auto D = DistTable(ins);
  ASSERT_EQ(get_makespan_lower_bound(ins, D), 11);
  auto info_m = std::string();
  const auto sol_m =
      solve(ins, info_m, 0, nullptr, nullptr, Objective::OBJ_MAKESPAN);
  ASSERT_EQ(get_makespan(sol_m), 11);
  ASSERT_EQ(get_info(info_m, "optimal"), "1");

  // the solution without waiting violates the STN
  ins.stn = nullptr;
  auto info2 = std::string();
  const auto sol2 = solve(ins, info2);
  ASSERT_EQ(get_makespan(sol2), 6);
  ins.stn = make_stn(ins, "timepoints 2\n0 0\n0 1\nconstraints 1\n1 2 5 -\n");
  ASSERT_FALSE(is_feasible_solution(ins, sol2));
}

// B registers its waypoint at least 3 steps after A
TEST(STN, cross_robot_precedence)
{
  auto ins =
      make_ins({xy(0, 0), xy(7, 7)}, {{xy(3, 0), xy(0, 0)}, {xy(7, 5), xy(7, 7)}});
  ins.stn = make_stn(ins, "timepoints 2\n0 0\n1 0\nconstraints 1\n1 2 3 -\n");
  ASSERT_TRUE(ins.stn->valid) << ins.stn->error;

  auto info = std::string();
  const auto sol =
      solve(ins, info, 0, nullptr, nullptr, Objective::OBJ_MAKESPAN);
  ASSERT_TRUE(is_feasible_solution(ins, sol, 1));
  const auto times = replay_events(ins, sol);
  ASSERT_EQ(times[0][0], 3);
  ASSERT_EQ(times[1][0], 6);  // arrived at t=2, waits for A
  ASSERT_EQ(get_makespan(sol), 8);

  // the lower bounds from the travel times and the absolute windows know that
  // B fires at 3 at the earliest (A at 0 at the earliest, plus 3), not at 6
  // (A arrives at 3); the ones with the temporal constraints do
  auto D = DistTable(ins);
  ASSERT_EQ(get_makespan_lower_bound(ins, D), 6);
  ASSERT_EQ(get_sum_of_costs_lower_bound(ins, D), 6 + 5);
  ASSERT_EQ(get_completion_lower_bounds(ins, D), std::vector<int>({6, 8}));

  // same constraint through a free timepoint (e.g., an operation)
  ins.stn = make_stn(ins,
                     "timepoints 3\n0 0\n-1 0\n1 0\n"
                     "constraints 2\n1 2 2 -\n2 3 1 -\n");
  ASSERT_TRUE(ins.stn->valid) << ins.stn->error;
  ASSERT_EQ(ins.stn->E, 3);
  ASSERT_EQ(ins.stn->D[2][1], -3);
  auto info2 = std::string();
  const auto sol2 =
      solve(ins, info2, 0, nullptr, nullptr, Objective::OBJ_MAKESPAN);
  ASSERT_EQ(replay_events(ins, sol2)[1][0], 6);
}

// agent 0 waits for agent 1, then agent 2 for agent 0: the earliest firing
// times need more than one pass over the agents
TEST(STN, completion_lower_bounds)
{
  auto ins = make_ins({xy(0, 0), xy(7, 7), xy(0, 7)},
                      {{xy(3, 0), xy(0, 0)},
                       {xy(7, 5), xy(7, 7)},
                       {xy(0, 6), xy(0, 7)}});
  // t(0,0) >= t(1,0) + 3, t(2,0) >= t(0,0)
  ins.stn = make_stn(ins,
                     "timepoints 3\n1 0\n0 0\n2 0\n"
                     "constraints 2\n1 2 3 -\n2 3 0 -\n");
  ASSERT_TRUE(ins.stn->valid) << ins.stn->error;
  auto D = DistTable(ins);
  // agent 1 fires at 2, agent 0 at 5 (arrived at 3), agent 2 at 5 (at 1)
  const auto C = get_completion_lower_bounds(ins, D);
  ASSERT_EQ(C, std::vector<int>({8, 4, 6}));

  auto info = std::string();
  const auto sol =
      solve(ins, info, 0, nullptr, nullptr, Objective::OBJ_MAKESPAN);
  ASSERT_TRUE(is_feasible_solution(ins, sol, 1));
  ASSERT_EQ(get_makespan(sol), 8);  // the bound is tight
  for (uint i = 0; i < ins.N; ++i)
    ASSERT_GE(get_path_cost(sol, i), C[i]);
  ASSERT_GE(get_sum_of_loss(ins, sol), 8 + 4 + 6);

  // without STN, the usual lower bounds
  ins.stn = nullptr;
  auto D2 = DistTable(ins);
  const auto C2 = get_completion_lower_bounds(ins, D2);
  ASSERT_EQ(C2, std::vector<int>({6, 4, 2}));
  ASSERT_EQ(get_sum_of_costs_lower_bound(ins, D2), 6 + 4 + 2);
  ASSERT_EQ(get_makespan_lower_bound(ins, D2), 6);
}

TEST(STN, deadlines)
{
  auto ins = make_ins({xy(0, 0)}, {{xy(7, 0), xy(0, 0)}});

  // unreachable: pruned at the start
  ins.stn = make_stn(ins, "timepoints 1\n0 0\nconstraints 1\n0 1 - 3\n");
  ASSERT_TRUE(ins.stn->valid) << ins.stn->error;
  ASSERT_TRUE(ins.stn->has_deadline);
  auto info = std::string();
  ASSERT_TRUE(solve(ins, info).empty());

  // reachable
  ins.stn = make_stn(ins, "timepoints 1\n0 0\nconstraints 1\n0 1 - 7\n");
  info.clear();
  auto sol = solve(ins, info);
  ASSERT_TRUE(is_feasible_solution(ins, sol, 1));
  ASSERT_EQ(replay_events(ins, sol)[0][0], 7);

  // exact time: arrives at 7, registers at 9
  ins.stn = make_stn(ins, "timepoints 1\n0 0\nconstraints 1\n0 1 9 9\n");
  info.clear();
  sol = solve(ins, info, 0, nullptr, nullptr, Objective::OBJ_MAKESPAN);
  ASSERT_TRUE(is_feasible_solution(ins, sol, 1));
  ASSERT_EQ(replay_events(ins, sol)[0][0], 9);
  ASSERT_EQ(get_makespan(sol), 16);
}

TEST(STN, load_errors)
{
  auto ins =
      make_ins({xy(0, 0), xy(7, 7)}, {{xy(3, 0), xy(0, 0)}, {xy(7, 5), xy(7, 7)}});
  auto valid = [&](const std::string& text) {
    return make_stn(ins, text)->valid;
  };
  ASSERT_TRUE(valid("# comment\ntimepoints 0\nconstraints 0\n"));
  ASSERT_TRUE(valid("timepoints 2\n0 0  # A\n1 0\nconstraints 1\n1 2 0 -\n"));
  ASSERT_FALSE(valid(""));
  ASSERT_FALSE(valid("timepoints 1\n0 0\nconstraints 1\n0 1 5 3\n"));  // inconsistent
  ASSERT_FALSE(valid("timepoints 2\n0 0\n1 0\nconstraints 1\n1 2 0 4\n"));  // upper bound
  ASSERT_FALSE(valid("timepoints 1\n0 1\nconstraints 0\n"));  // final waypoint
  ASSERT_FALSE(valid("timepoints 1\n2 0\nconstraints 0\n"));  // no agent 2
  ASSERT_FALSE(valid("timepoints 2\n0 0\n0 0\nconstraints 0\n"));  // duplicated
  ASSERT_FALSE(valid("timepoints 1\n0 0\nconstraints 1\n0 2 0 -\n"));  // no tp 2
  ASSERT_FALSE(valid("timepoints 1\n0 0\nconstraints 1\n0 1 x -\n"));
  ASSERT_FALSE(valid("timepoints 1\n0 0\nconstraints 0\n7\n"));  // trailing
  ASSERT_FALSE(
      TemporalNetwork("./assets/not-found.stn", ins).valid);
}

// an STN without constraints changes nothing: same search
TEST(STN, no_constraints_equals_waypoints)
{
  const auto map = "./assets/random-32-32-10.map";
  auto MT = std::mt19937(0);
  auto ins = Instance(map, &MT, 20, 3, true);
  ASSERT_TRUE(ins.is_valid());

  auto run = [&](std::string& info) {
    auto MT_s = std::mt19937(0);
    return solve(ins, info, 0, nullptr, &MT_s);
  };
  auto info1 = std::string(), info2 = std::string();
  const auto s1 = run(info1);
  auto text = std::string("timepoints ") + std::to_string(ins.N * 3) + "\n";
  for (uint i = 0; i < ins.N; ++i)
    for (uint k = 0; k < 3; ++k)
      text += std::to_string(i) + " " + std::to_string(k) + "\n";
  ins.stn = make_stn(ins, text + "constraints 0\n");
  ASSERT_TRUE(ins.stn->valid) << ins.stn->error;
  const auto s2 = run(info2);
  ASSERT_FALSE(s1.empty());
  ASSERT_EQ(get_info(info1, "num_node_gen"), get_info(info2, "num_node_gen"));
  ASSERT_EQ(s1.size(), s2.size());
  for (size_t t = 0; t < s1.size(); ++t)
    ASSERT_TRUE(is_same_config(s1[t], s2[t]));
}

TEST(STN, private_depots)
{
  // agent 1 leaves its depot (1,0); agent 0 must not cross it
  auto ins = make_ins({xy(0, 0), xy(1, 0)},
                      {{xy(2, 0)}, {xy(1, 3), xy(1, 0)}});
  ins.set_private_depots();
  ASSERT_TRUE(ins.is_valid());
  ASSERT_TRUE(ins.forbidden(0, ins.G.U[xy(1, 0)]));
  ASSERT_FALSE(ins.forbidden(1, ins.G.U[xy(1, 0)]));
  auto info = std::string();
  const auto sol =
      solve(ins, info, 0, nullptr, nullptr, Objective::OBJ_MAKESPAN);
  ASSERT_TRUE(is_feasible_solution(ins, sol, 1));
  for (auto& C : sol) ASSERT_NE(C[0], ins.G.U[xy(1, 0)]);

  // a waypoint on the depot of another agent
  auto ins2 = make_ins({xy(0, 0), xy(1, 0)},
                       {{xy(1, 0), xy(2, 0)}, {xy(1, 3), xy(1, 0)}});
  ins2.set_private_depots();
  ASSERT_FALSE(ins2.is_valid());
}

// precedences among agents in a chain, random instances
TEST(STN, random_chain)
{
  const auto map = "./assets/random-32-32-10.map";
  for (uint seed = 0; seed < 3; ++seed) {
    auto MT = std::mt19937(seed);
    const uint N = 10, m = 3;
    auto ins = Instance(map, &MT, N, m, true);
    ASSERT_TRUE(ins.is_valid());
    // event (i, k): timepoint 1 + i * m + k
    // (i+1, 0) at least 2 steps after (i, 0); (i, 1) at least 4 after (i, 0)
    auto text = "timepoints " + std::to_string(N * m) + "\n";
    for (uint i = 0; i < N; ++i)
      for (uint k = 0; k < m; ++k)
        text += std::to_string(i) + " " + std::to_string(k) + "\n";
    auto cons = std::vector<std::string>();
    for (uint i = 0; i < N; ++i) {
      const auto e0 = 1 + i * m;
      cons.push_back(std::to_string(e0) + " " + std::to_string(e0 + 1) + " 4 -");
      if (i + 1 < N)
        cons.push_back(std::to_string(e0) + " " + std::to_string(e0 + m) +
                       " 2 -");
    }
    text += "constraints " + std::to_string(cons.size()) + "\n";
    for (auto& c : cons) text += c + "\n";
    ins.stn = make_stn(ins, text);
    ASSERT_TRUE(ins.stn->valid) << ins.stn->error;

    auto info = std::string();
    const auto deadline = Deadline(5000);
    const auto sol = solve(ins, info, 0, &deadline, &MT);
    ASSERT_FALSE(sol.empty()) << "seed=" << seed;
    ASSERT_TRUE(is_feasible_solution(ins, sol, 1));

    auto info_m = std::string();
    const auto deadline_m = Deadline(1000);
    const auto sol_m = solve(ins, info_m, 0, &deadline_m, &MT, OBJ_MAKESPAN);
    ASSERT_TRUE(is_feasible_solution(ins, sol_m, 1));
    auto D = DistTable(ins);
    ASSERT_GE(get_makespan(sol_m), get_makespan_lower_bound(ins, D));
  }
}
