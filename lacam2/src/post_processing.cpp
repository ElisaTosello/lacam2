#include "../include/post_processing.hpp"

#include "../include/dist_table.hpp"
#include "../include/stn.hpp"

std::vector<std::vector<int> > replay_events(const Instance& ins,
                                             const Solution& solution,
                                             bool* alive)
{
  auto times = std::vector<std::vector<int> >(ins.N);
  for (uint i = 0; i < ins.N; ++i) times[i].assign(ins.m(i), -1);
  if (alive != nullptr) *alive = true;
  if (solution.empty()) return times;
  auto K = std::vector<uint>();
  auto W = Windows();
  auto record = [&](const std::vector<uint>& K_old, int t) {
    for (uint i = 0; i < ins.N; ++i)
      for (auto k = K_old[i]; k < K[i]; ++k) times[i][k] = t;
  };
  bool ok = initial_state(ins, K, W);
  record(std::vector<uint>(ins.N, 0), 0);
  for (size_t t = 1; t < solution.size(); ++t) {
    const auto K_old = K;
    ok &= advance_state(ins, solution[t], K, W);
    record(K_old, t);
  }
  // final waypoints: when the agent arrives there for the last time
  for (uint i = 0; i < ins.N; ++i) {
    auto t = (int)solution.size() - 1;
    while (t > 0 && solution[t - 1][i] == ins.goals[i]) --t;
    if (ins.intermediates_done(K, i)) times[i].back() = t;
  }
  if (alive != nullptr) *alive = ok;
  return times;
}

bool is_feasible_solution(const Instance& ins, const Solution& solution,
                          const int verbose)
{
  if (solution.empty()) return true;

  // check start locations
  if (!is_same_config(solution.front(), ins.starts)) {
    info(1, verbose, "invalid starts");
    return false;
  }

  // check waypoint sequences, in order, and temporal constraints
  bool alive;
  const auto times = replay_events(ins, solution, &alive);
  if (!alive) {
    info(1, verbose, "missed deadline");
    return false;
  }
  for (size_t i = 0; i < ins.N; ++i) {
    for (size_t k = 0; k + 1 < ins.m(i); ++k) {
      if (times[i][k] < 0) {
        info(1, verbose, "agent ", i, " did not complete its waypoints, ", k,
             "/", ins.m(i) - 1);
        return false;
      }
    }
  }
  if (ins.stn != nullptr) {
    const auto& S = *ins.stn;
    auto t_of = [&](uint e) { return e == 0 ? 0 : times[S.agent[e]][S.index[e]]; };
    for (uint a = 0; a < S.E; ++a)
      for (uint b = 0; b < S.E; ++b) {
        if (S.D[a][b] < TemporalNetwork::INF && t_of(b) - t_of(a) > S.D[a][b]) {
          info(1, verbose, "temporal constraint violated between events ", a,
               " and ", b);
          return false;
        }
      }
  }

  // check goal locations
  if (!is_same_config(solution.back(), ins.goals)) {
    info(1, verbose, "invalid goals");
    return false;
  }

  for (size_t t = 1; t < solution.size(); ++t) {
    for (size_t i = 0; i < ins.N; ++i) {
      auto v_i_from = solution[t - 1][i];
      auto v_i_to = solution[t][i];
      // check reserved vertices
      if (ins.forbidden(i, v_i_to)) {
        info(1, verbose, "agent ", i, " on a vertex reserved for another");
        return false;
      }
      // check connectivity
      if (v_i_from != v_i_to &&
          std::find(v_i_to->neighbor.begin(), v_i_to->neighbor.end(),
                    v_i_from) == v_i_to->neighbor.end()) {
        info(1, verbose, "invalid move");
        return false;
      }

      // check conflicts
      for (size_t j = i + 1; j < ins.N; ++j) {
        auto v_j_from = solution[t - 1][j];
        auto v_j_to = solution[t][j];
        // vertex conflicts
        if (v_j_to == v_i_to) {
          info(1, verbose, "vertex conflict");
          return false;
        }
        // swap conflicts
        if (v_j_to == v_i_from && v_j_from == v_i_to) {
          info(1, verbose, "edge conflict");
          return false;
        }
      }
    }
  }

  return true;
}

int get_makespan(const Solution& solution)
{
  if (solution.empty()) return 0;
  return solution.size() - 1;
}

int get_path_cost(const Solution& solution, uint i)
{
  const auto makespan = solution.size();
  const auto g = solution.back()[i];
  auto c = makespan;
  while (c > 0 && solution[c - 1][i] == g) --c;
  return c;
}

int get_sum_of_costs(const Solution& solution)
{
  if (solution.empty()) return 0;
  int c = 0;
  const auto N = solution.front().size();
  for (size_t i = 0; i < N; ++i) c += get_path_cost(solution, i);
  return c;
}

int get_sum_of_loss(const Solution& solution)
{
  if (solution.empty()) return 0;
  int c = 0;
  const auto N = solution.front().size();
  const auto T = solution.size();
  for (size_t i = 0; i < N; ++i) {
    auto g = solution.back()[i];
    for (size_t t = 1; t < T; ++t) {
      if (solution[t - 1][i] != g || solution[t][i] != g) ++c;
    }
  }
  return c;
}

int get_sum_of_loss(const Instance& ins, const Solution& solution)
{
  if (solution.empty()) return 0;
  int c = 0;
  auto K = std::vector<uint>();
  auto W = Windows();
  initial_state(ins, K, W);
  for (size_t t = 1; t < solution.size(); ++t) {
    for (size_t i = 0; i < ins.N; ++i) {
      const auto g = ins.goals[i];
      if (!ins.intermediates_done(K, i) || solution[t - 1][i] != g ||
          solution[t][i] != g)
        ++c;
    }
    advance_state(ins, solution[t], K, W);
  }
  return c;
}

int get_makespan_lower_bound(const Instance& ins, DistTable& dist_table)
{
  uint c = 0;
  auto K = std::vector<uint>();
  auto W = Windows();
  initial_state(ins, K, W);
  for (size_t i = 0; i < ins.N; ++i) {
    c = std::max(c, time_to_go(ins, dist_table, K, W, i, ins.starts[i]));
  }
  return c;
}

int get_sum_of_costs_lower_bound(const Instance& ins, DistTable& dist_table)
{
  int c = 0;
  auto K = std::vector<uint>();
  auto W = Windows();
  initial_state(ins, K, W);
  for (size_t i = 0; i < ins.N; ++i) {
    c += time_to_go(ins, dist_table, K, W, i, ins.starts[i]);
  }
  return c;
}

std::vector<int> get_completion_lower_bounds(const Instance& ins,
                                             DistTable& dist_table)
{
  const auto S = ins.stn.get();  // nullptr: no temporal constraint
  const long long INF = TemporalNetwork::INF;
  // earliest firing time of each event (origin: 0), raised until a fixed
  // point; it is reached since the network is consistent, i.e., it has no
  // positive cycle
  auto est = std::vector<long long>(S != nullptr ? S->E : 0, 0);
  auto C = std::vector<int>(ins.N, 0);
  bool changed = true;
  while (changed) {
    changed = false;
    for (uint i = 0; i < ins.N; ++i) {
      long long tau = 0;
      auto pos = ins.starts[i];
      for (uint k = 0; k < ins.m(i); ++k) {
        tau += dist_table.get_wp(i, k, pos);  // arrival at waypoint k
        pos = ins.waypoints[i][k];
        const int e =
            (S != nullptr && k + 1 < ins.m(i)) ? S->event_of[i][k] : -1;
        if (e <= 0) continue;
        // it waits there until t_e >= t_x - D[e][x] for every timepoint x:
        // precedences, delays, release times (x = origin)
        for (uint x = 0; x < S->E; ++x) {
          if ((int)x != e && S->D[e][x] < INF)
            tau = std::max(tau, est[x] - S->D[e][x]);
        }
        tau = std::max(tau, est[e]);
        if (tau > est[e]) {
          est[e] = tau;
          changed = true;
        }
      }
      C[i] = tau;
    }
  }
  return C;
}

void print_stats(const int verbose, const Instance& ins,
                 const Solution& solution, const double comp_time_ms)
{
  auto ceil = [](float x) { return std::ceil(x * 100) / 100; };
  auto dist_table = DistTable(ins);
  const auto makespan = get_makespan(solution);
  const auto makespan_lb = get_makespan_lower_bound(ins, dist_table);
  const auto sum_of_costs = get_sum_of_costs(solution);
  const auto sum_of_costs_lb = get_sum_of_costs_lower_bound(ins, dist_table);
  const auto sum_of_loss = get_sum_of_loss(ins, solution);
  info(1, verbose, "solved: ", comp_time_ms, "ms", "\tmakespan: ", makespan,
       " (lb=", makespan_lb, ", ub=", ceil((float)makespan / makespan_lb), ")",
       "\tsum_of_costs: ", sum_of_costs, " (lb=", sum_of_costs_lb,
       ", ub=", ceil((float)sum_of_costs / sum_of_costs_lb), ")",
       "\tsum_of_loss: ", sum_of_loss, " (lb=", sum_of_costs_lb,
       ", ub=", ceil((float)sum_of_loss / sum_of_costs_lb), ")");
}

// for log of map_name
static const std::regex r_map_name = std::regex(R"(.+/(.+))");

void make_log(const Instance& ins, const Solution& solution,
              const std::string& output_name, const double comp_time_ms,
              const std::string& map_name, const int seed,
              const std::string& additional_info, const bool log_short)
{
  // map name
  std::smatch results;
  const auto map_recorded_name =
      (std::regex_match(map_name, results, r_map_name)) ? results[1].str()
                                                        : map_name;

  // for instance-specific values
  auto dist_table = DistTable(ins);

  // log for visualizer
  auto get_x = [&](int k) { return k % ins.G.width; };
  auto get_y = [&](int k) { return k / ins.G.width; };
  std::ofstream log;
  log.open(output_name, std::ios::out);
  log << "agents=" << ins.N << "\n";
  log << "map_file=" << map_recorded_name << "\n";
  log << "solver=planner\n";
  log << "solved=" << !solution.empty() << "\n";
  log << "soc=" << get_sum_of_costs(solution) << "\n";
  log << "soc_lb=" << get_sum_of_costs_lower_bound(ins, dist_table) << "\n";
  log << "makespan=" << get_makespan(solution) << "\n";
  log << "makespan_lb=" << get_makespan_lower_bound(ins, dist_table) << "\n";
  log << "sum_of_loss=" << get_sum_of_loss(ins, solution) << "\n";
  log << "sum_of_loss_lb=" << get_sum_of_costs_lower_bound(ins, dist_table)
      << "\n";
  {
    // also with the waits imposed by the temporal constraints
    const auto C = get_completion_lower_bounds(ins, dist_table);
    long long soc = 0;
    int makespan = 0;
    for (auto c : C) {
      soc += c;
      makespan = std::max(makespan, c);
    }
    log << "soc_lb_stn=" << soc << "\n";
    log << "makespan_lb_stn=" << makespan << "\n";
  }
  log << "comp_time=" << comp_time_ms << "\n";
  log << "seed=" << seed << "\n";
  log << additional_info;
  if (log_short) return;
  log << "starts=";
  for (size_t i = 0; i < ins.N; ++i) {
    auto k = ins.starts[i]->index;
    log << "(" << get_x(k) << "," << get_y(k) << "),";
  }
  log << "\ngoals=";
  for (size_t i = 0; i < ins.N; ++i) {
    auto k = ins.goals[i]->index;
    log << "(" << get_x(k) << "," << get_y(k) << "),";
  }
  if (ins.has_waypoints()) {
    // per agent: (x,y);(x,y);... separated by '|'
    log << "\nwaypoints=";
    for (size_t i = 0; i < ins.N; ++i) {
      if (i > 0) log << "|";
      for (size_t k = 0; k < ins.m(i); ++k) {
        auto l = ins.waypoints[i][k]->index;
        if (k > 0) log << ";";
        log << "(" << get_x(l) << "," << get_y(l) << ")";
      }
    }
  }
  if (ins.stn != nullptr && !solution.empty()) {
    // per agent: time at which each waypoint is registered, separated by '|'
    const auto times = replay_events(ins, solution);
    log << "\nevents=";
    for (size_t i = 0; i < ins.N; ++i) {
      if (i > 0) log << "|";
      for (size_t k = 0; k < ins.m(i); ++k) log << (k > 0 ? ";" : "") << times[i][k];
    }
  }
  log << "\nsolution=\n";
  for (size_t t = 0; t < solution.size(); ++t) {
    log << t << ":";
    auto C = solution[t];
    for (auto v : C) {
      log << "(" << get_x(v->index) << "," << get_y(v->index) << "),";
    }
    log << "\n";
  }
  log.close();
}
