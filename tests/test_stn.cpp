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
  auto info_s = std::string();
  const auto sol_s = solve(ins, info_s, 0, nullptr, nullptr, OBJ_MAKESPAN,
                           0.001, WP_RESET_EACH, PRIO_SLACK);
  ASSERT_EQ(get_makespan(sol_s), 11);
  ASSERT_EQ(get_info(info_s, "optimal"), "1");
  ASSERT_EQ(get_info(info_s, "target_makespan"), "11");

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

  // slack-based order: same optimum, target makespan = max of the bounds
  auto info_s = std::string();
  const auto sol_s = solve(ins, info_s, 0, nullptr, nullptr, OBJ_MAKESPAN,
                           0.001, WP_RESET_EACH, PRIO_SLACK);
  ASSERT_TRUE(is_feasible_solution(ins, sol_s, 1));
  ASSERT_EQ(get_makespan(sol_s), 8);
  ASSERT_EQ(get_info(info_s, "optimal"), "1");
  ASSERT_EQ(get_info(info_s, "priority"), "slack");
  ASSERT_EQ(get_info(info_s, "target_makespan"), "8");
  ASSERT_EQ(get_info(info, "priority"), "");  // default output unchanged
  auto info_r = std::string();
  const auto sol_r = solve(ins, info_r, 0, nullptr, nullptr, OBJ_MAKESPAN,
                           0.001, WP_RESET_EACH, PRIO_SLACK_RELEASE);
  ASSERT_TRUE(is_feasible_solution(ins, sol_r, 1));
  ASSERT_EQ(get_makespan(sol_r), 8);
  ASSERT_EQ(get_info(info_r, "optimal"), "1");
  ASSERT_EQ(get_info(info_r, "priority"), "slack_release");

  // without STN, the usual lower bounds
  ins.stn = nullptr;
  auto D2 = DistTable(ins);
  const auto C2 = get_completion_lower_bounds(ins, D2);
  ASSERT_EQ(C2, std::vector<int>({6, 4, 2}));
  ASSERT_EQ(get_sum_of_costs_lower_bound(ins, D2), 6 + 4 + 2);
  ASSERT_EQ(get_makespan_lower_bound(ins, D2), 6);
}

// same instance as completion_lower_bounds: target makespan T* = 8
static Instance make_chain_ins()
{
  auto ins = make_ins({xy(0, 0), xy(7, 7), xy(0, 7)},
                      {{xy(3, 0), xy(0, 0)},
                       {xy(7, 5), xy(7, 7)},
                       {xy(0, 6), xy(0, 7)}});
  // t(0,0) >= t(1,0) + 3, t(2,0) >= t(0,0)
  ins.stn = make_stn(ins,
                     "timepoints 3\n1 0\n0 0\n2 0\n"
                     "constraints 2\n1 2 3 -\n2 3 0 -\n");
  return ins;
}

TEST(STN, waypoint_deadlines)
{
  using DL = std::vector<std::vector<int> >;
  auto ins = make_chain_ins();
  ASSERT_TRUE(ins.stn->valid) << ins.stn->error;
  auto D = DistTable(ins);
  // agent 0 registers by 8 - 3 (it returns in 3); agent 1 by 5 - 3, since
  // agent 0 registers at least 3 later; agent 2 by 8 - 1
  ASSERT_EQ(waypoint_deadlines(ins, D, 8), DL({{5, 8}, {2, 8}, {7, 8}}));

  // without STN, T* = 6: the remaining travel only, T* - rem
  ins.stn = nullptr;
  ASSERT_EQ(waypoint_deadlines(ins, D, 6), DL({{3, 6}, {4, 6}, {5, 6}}));

  // drop then pickup on the same cell, 5 apart: T* = 11, the pickup by
  // 11 - 3, the drop by 8 - 5
  const auto X = xy(3, 0);
  auto ins2 = make_ins({xy(0, 0)}, {{X, X, xy(0, 0)}});
  ins2.stn = make_stn(ins2, "timepoints 2\n0 0\n0 1\nconstraints 1\n1 2 5 -\n");
  ASSERT_TRUE(ins2.stn->valid) << ins2.stn->error;
  auto D2 = DistTable(ins2);
  ASSERT_EQ(get_makespan_lower_bound(ins2, D2), 11);
  ASSERT_EQ(waypoint_deadlines(ins2, D2, 11), DL({{3, 8, 11}}));

  // absolute deadline tighter than T*
  ins2.stn = make_stn(ins2,
                      "timepoints 2\n0 0\n0 1\nconstraints 2\n1 2 5 -\n0 1 - 4\n");
  ASSERT_TRUE(ins2.stn->valid) << ins2.stn->error;
  ASSERT_EQ(waypoint_deadlines(ins2, D2, 11), DL({{3, 8, 11}}));
  ins2.stn = make_stn(ins2,
                      "timepoints 2\n0 0\n0 1\nconstraints 2\n1 2 5 -\n0 1 - 2\n");
  ASSERT_TRUE(ins2.stn->valid) << ins2.stn->error;
  ASSERT_EQ(waypoint_deadlines(ins2, D2, 11), DL({{2, 8, 11}}));
}

// order of the agents along the optimal plan of the chain instance: slack =
// deadline of the next waypoint - (t + distance to it), agents done last
TEST(STN, slack_priorities)
{
  auto ins = make_chain_ins();
  ASSERT_TRUE(ins.stn->valid) << ins.stn->error;
  auto D = DistTable(ins);
  const auto dl = waypoint_deadlines(ins, D, 8);
  const float DONE = -1e9;
  auto config = [&](std::vector<uint> idx) {
    auto C = Config();
    for (auto k : idx) C.push_back(ins.G.U[k]);
    return C;
  };
  // configuration at t, slack (or DONE), order; the same with release
  struct Step {
    Config C;
    std::vector<float> slack;
    std::vector<uint> order;
    std::vector<float> slack_rel;
  };
  const auto steps = std::vector<Step>({
      // with release, agents 0 and 2 are blocked (predecessor not fired)
      {config({xy(0, 0), xy(7, 7), xy(0, 7)}), {2, 0, 6}, {1, 0, 2}, {2, 0, 6}},
      {config({xy(1, 0), xy(7, 6), xy(0, 6)}), {2, 0, 6}, {1, 0, 2}, {2, 0, 6}},
      // agent 1 registers: deadline of its final goal; with release, agent
      // 0 cannot register before t = 5, i.e., in 3 steps instead of 1
      {config({xy(2, 0), xy(7, 5), xy(0, 6)}), {2, 4, 5}, {0, 1, 2}, {0, 4, 5}},
      // agents 1 and 2 tie: order of the parent
      {config({xy(3, 0), xy(7, 6), xy(0, 6)}), {2, 4, 4}, {0, 1, 2}, {0, 4, 4}},
      // waiting agents lose one per step, not with a known release (agent 0)
      {config({xy(3, 0), xy(7, 7), xy(0, 6)}), {1, DONE, 3}, {0, 2, 1},
       {0, DONE, 3}},
      // agents 0 and 2 register
      {config({xy(3, 0), xy(7, 7), xy(0, 6)}), {0, DONE, 2}, {0, 2, 1},
       {0, DONE, 2}},
  });

  for (bool release : {false, true}) {
    auto K = std::vector<uint>();
    auto W = Windows();
    ASSERT_TRUE(initial_state(ins, K, W));
    auto nodes = std::vector<std::unique_ptr<HNode> >();
    for (uint t = 0; t < steps.size(); ++t) {
      if (t > 0) ASSERT_TRUE(advance_state(ins, steps[t].C, K, W));
      prune_windows(ins, D, steps[t].C, K, W);  // as in the planner
      HNode* parent = t > 0 ? nodes.back().get() : nullptr;
      nodes.emplace_back(new HNode(steps[t].C, K, W, &ins, D, parent, t, 0,
                                   WP_RESET_EACH, &dl, release));
      const auto H = nodes.back().get();
      ASSERT_EQ(H->t, t);
      auto slack = H->priorities;
      for (auto& s : slack)
        if (s != DONE) s = -s;
      ASSERT_EQ(slack, release ? steps[t].slack_rel : steps[t].slack)
          << "t=" << t << " release=" << release;
      ASSERT_EQ(H->order, steps[t].order) << "t=" << t;
    }
    ASSERT_EQ(K, std::vector<uint>({1, 1, 1}));  // all registered at t = 5
  }

  // distance-based at the root: the longest way to go first
  auto K0 = std::vector<uint>();
  auto W0 = Windows();
  initial_state(ins, K0, W0);
  const auto H0 = HNode(ins.starts, K0, W0, &ins, D, nullptr, 0, 0);
  ASSERT_EQ(H0.order, std::vector<uint>({0, 1, 2}));
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

    for (auto prio : {PRIO_DIST, PRIO_SLACK, PRIO_SLACK_RELEASE}) {
      auto info = std::string();
      const auto deadline = Deadline(5000);
      const auto sol = solve(ins, info, 0, &deadline, &MT, OBJ_NONE, 0.001,
                             WP_RESET_EACH, prio);
      ASSERT_FALSE(sol.empty()) << "seed=" << seed << " priority=" << prio;
      ASSERT_TRUE(is_feasible_solution(ins, sol, 1));

      auto info_m = std::string();
      const auto deadline_m = Deadline(1000);
      const auto sol_m = solve(ins, info_m, 0, &deadline_m, &MT, OBJ_MAKESPAN,
                               0.001, WP_RESET_EACH, prio);
      ASSERT_TRUE(is_feasible_solution(ins, sol_m, 1));
      auto D = DistTable(ins);
      ASSERT_GE(get_makespan(sol_m), get_makespan_lower_bound(ins, D));
    }
  }
}
