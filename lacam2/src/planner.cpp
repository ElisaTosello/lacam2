#include "../include/planner.hpp"

LNode::LNode(LNodePtr _parent, uint i, Vertex* v)
    : parent(_parent),
      who(i),
      where(v),
      depth(_parent == nullptr ? 0 : _parent->depth + 1)
{
}

uint hash_state(const Config& C, const std::vector<uint>& K,
                const Windows& W)
{
  uint hash = ConfigHasher()(C);
  for (auto k : K) hash ^= k + 0x9e3779b9 + (hash << 6) + (hash >> 2);
  return W.empty() ? hash : hash_windows(W, hash);
}

HNode* Explored::find(const Config& C, const std::vector<uint>& K,
                      const Windows& W, uint hash) const
{
  const auto range = table.equal_range(hash);
  for (auto it = range.first; it != range.second; ++it) {
    const auto H = it->second;
    if (H->C == C && H->K == K && H->W == W) return H;
  }
  return nullptr;
}

uint HNode::HNODE_CNT = 0;

// for high-level
HNode::HNode(const Config& _C, const std::vector<uint>& _K,
             const Windows& _W, const Instance* ins, DistTable& D,
             HNode* _parent, const uint _g, const uint _h,
             const PriorityReset reset)
    : C(_C),
      K(_K),
      W(_W),
      parent(_parent),
      neighbor(),
      g(_g),
      h(_h),
      f(g + h),
      priorities(C.size()),
      order(C.size(), 0),
      search_tree(std::queue<LNodePtr>())
{
  ++HNODE_CNT;

  search_tree.push(std::make_shared<LNode>());
  const auto N = C.size();

  // update neighbor
  if (parent != nullptr) parent->neighbor.insert(this);

  // set priorities
  if (parent == nullptr) {
    // initialize
    for (uint i = 0; i < N; ++i) priorities[i] = (float)D.h(i, K[i], C[i]) / N;
  } else {
    // dynamic priorities, akin to PIBT
    for (size_t i = 0; i < N; ++i) {
      const bool done = ins->intermediates_done(K, i) && C[i] == ins->goals[i];
      const bool reached_wp =
          reset == WP_RESET_EACH && K[i] > parent->K[i];
      // with STN: an agent that would arrive too early does not gain
      // priority, it yields to the others
      const bool early =
          ins->stn != nullptr && !done &&
          event_release(*ins, K, W, i) > (int)D.get_wp(i, K[i], C[i]);
      if (early) {
        priorities[i] = parent->priorities[i];
      } else if (!done && !reached_wp) {
        priorities[i] = parent->priorities[i] + 1;
      } else {
        priorities[i] = parent->priorities[i] - (int)parent->priorities[i];
      }
    }
  }

  // set order
  std::iota(order.begin(), order.end(), 0);
  std::sort(order.begin(), order.end(),
            [&](uint i, uint j) { return priorities[i] > priorities[j]; });
}

HNode::~HNode()
{
  // LNodes are freed by shared_ptr
}

Planner::Planner(const Instance* _ins, const Deadline* _deadline,
                 std::mt19937* _MT, const int _verbose,
                 const Objective _objective, const float _restart_rate,
                 const PriorityReset _wp_reset)
    : ins(_ins),
      deadline(_deadline),
      MT(_MT),
      verbose(_verbose),
      objective(_objective),
      RESTART_RATE(_restart_rate),
      WP_RESET(_wp_reset),
      N(ins->N),
      V_size(ins->G.size()),
      D(DistTable(ins)),
      loop_cnt(0),
      C_next(N),
      tie_breakers(V_size, 0),
      A(N, nullptr),
      occupied_now(V_size, nullptr),
      occupied_next(V_size, nullptr),
      K_cur(nullptr),
      slot_cur(N, 0),
      K_last([&] {
        auto K = std::vector<uint>(N);
        for (uint i = 0; i < N; ++i) K[i] = ins->m(i) - 1;
        return K;
      }())
{
}

Planner::~Planner() {}

Solution Planner::solve(std::string& additional_info)
{
  solver_info(1, "start search");

  // setup agents
  for (auto i = 0; i < N; ++i) A[i] = new Agent(i);

  // setup search
  auto OPEN = std::stack<HNode*>();
  // key: (configuration, waypoint progress)
  // hashing only configurations would make the search incomplete,
  // e.g., when returning to a depot visited before
  auto EXPLORED = Explored();
  // insert initial node, 'H': high-level node
  auto K_init = std::vector<uint>();
  auto W_init = Windows();
  bool init_alive = initial_state(*ins, K_init, W_init);
  prune_windows(*ins, D, ins->starts, K_init, W_init);
  init_alive = init_alive && meets_deadlines(ins->starts, K_init, W_init);
  auto H_init =
      new HNode(ins->starts, K_init, W_init, ins, D, nullptr, 0,
                get_h_value(ins->starts, K_init, W_init), WP_RESET);
  if (init_alive) OPEN.push(H_init);  // otherwise, no solution
  EXPLORED.insert(H_init, hash_state(H_init->C, H_init->K, H_init->W));

  std::vector<Config> solution;
  auto C_new = Config(N, nullptr);         // for new configuration
  auto K_new = std::vector<uint>(N, 0);    // for new waypoint progress
  auto W_new = Windows();                  // for new time windows
  HNode* H_goal = nullptr;          // to store goal node

  // DFS
  while (!OPEN.empty() && !is_expired(deadline)) {
    loop_cnt += 1;

    // do not pop here!
    auto H = OPEN.top();  // high-level node

    // low-level search end
    if (H->search_tree.empty()) {
      OPEN.pop();
      continue;
    }

    // check lower bounds
    if (H_goal != nullptr && H->f >= H_goal->f) {
      OPEN.pop();
      continue;
    }

    // check goal condition
    if (H_goal == nullptr && is_goal(H->C, H->K)) {
      H_goal = H;
      solver_info(1, "found solution, cost: ", H->g);
      log_anytime(H_goal);
      if (objective == OBJ_NONE) break;
      continue;
    }

    // create successors at the low-level search
    auto L = H->search_tree.front();
    H->search_tree.pop();
    expand_lowlevel_tree(H, L);

    // create successors at the high-level search
    const auto res = get_new_config(H, L);
    L.reset();  // free, unless shared with children
    if (!res) continue;

    // create new configuration
    for (auto a : A) C_new[a->id] = a->v_next;
    K_new = H->K;
    W_new = H->W;
    if (!advance_state(*ins, C_new, K_new, W_new)) continue;  // missed deadline
    prune_windows(*ins, D, C_new, K_new, W_new);  // merge equivalent states
    if (!meets_deadlines(C_new, K_new, W_new)) continue;

    // check explored list
    const auto hash_new = hash_state(C_new, K_new, W_new);
    const auto H_found = EXPLORED.find(C_new, K_new, W_new, hash_new);
    if (H_found != nullptr) {
      // case found
      rewrite(H, H_found, H_goal, OPEN);
      // re-insert or random-restart
      auto H_insert = (MT != nullptr && get_random_float(MT) >= RESTART_RATE)
                          ? H_found
                          : H_init;
      if (H_goal == nullptr || H_insert->f < H_goal->f) OPEN.push(H_insert);
    } else {
      // insert new search node
      const auto H_new = new HNode(
          C_new, K_new, W_new, ins, D, H, H->g + get_edge_cost(H, C_new),
          get_h_value(C_new, K_new, W_new), WP_RESET);
      EXPLORED.insert(H_new, hash_new);
      if (H_goal == nullptr || H_new->f < H_goal->f) OPEN.push(H_new);
    }
  }

  // backtrack
  if (H_goal != nullptr) {
    auto H = H_goal;
    while (H != nullptr) {
      solution.push_back(H->C);
      H = H->parent;
    }
    std::reverse(solution.begin(), solution.end());
  }

  // print result
  if (H_goal != nullptr && OPEN.empty()) {
    solver_info(1, "solved optimally, objective: ", objective);
  } else if (H_goal != nullptr) {
    solver_info(1, "solved sub-optimally, objective: ", objective);
  } else if (OPEN.empty()) {
    solver_info(1, "no solution");
  } else if (deadline != nullptr && deadline->memory_exceeded) {
    solver_info(1, "memory limit");
  } else {
    solver_info(1, "timeout");
  }

  // why the search ended: search space exhausted, first solution (no
  // objective), memory limit or time limit
  const auto stopped =
      OPEN.empty()                                     ? "complete"
      : (H_goal != nullptr && objective == OBJ_NONE)   ? "first_solution"
      : (deadline != nullptr && deadline->memory_exceeded) ? "memory"
                                                           : "time";

  // logging
  additional_info += "stopped=" + std::string(stopped) + "\n";
  additional_info +=
      "optimal=" + std::to_string(H_goal != nullptr && OPEN.empty()) + "\n";
  additional_info += "objective=" + std::to_string(objective) + "\n";
  additional_info += "loop_cnt=" + std::to_string(loop_cnt) + "\n";
  additional_info += "num_node_gen=" + std::to_string(EXPLORED.size()) + "\n";
  additional_info += "anytime=" + anytime_log + "\n";

  // memory management
  for (auto a : A) delete a;
  for (auto itr : EXPLORED.table) delete itr.second;

  return solution;
}

void Planner::log_anytime(HNode* H_goal)
{
  uint makespan = 0;
  for (auto H = H_goal->parent; H != nullptr; H = H->parent) ++makespan;
  if (!anytime_log.empty()) anytime_log += ",";
  anytime_log += std::to_string(elapsed_ms(deadline)) + ":" +
                 std::to_string(H_goal->g) + ":" + std::to_string(makespan);
}

void Planner::rewrite(HNode* H_from, HNode* H_to, HNode* H_goal,
                      std::stack<HNode*>& OPEN)
{
  // update neighbors
  H_from->neighbor.insert(H_to);

  // Dijkstra update
  std::queue<HNode*> Q({H_from});  // queue is sufficient
  while (!Q.empty()) {
    auto n_from = Q.front();
    Q.pop();
    for (auto n_to : n_from->neighbor) {
      auto g_val = n_from->g + get_edge_cost(n_from, n_to);
      if (g_val < n_to->g) {
        if (n_to == H_goal)
          solver_info(1, "cost update: ", n_to->g, " -> ", g_val);
        n_to->g = g_val;
        n_to->f = n_to->g + n_to->h;
        n_to->parent = n_from;
        if (n_to == H_goal) log_anytime(H_goal);
        Q.push(n_to);
        if (H_goal != nullptr && n_to->f < H_goal->f) OPEN.push(n_to);
      }
    }
  }
}

uint Planner::get_edge_cost(HNode* H_from, const Config& C2)
{
  if (objective == OBJ_SUM_OF_LOSS) {
    const auto& C1 = H_from->C;
    const auto& K1 = H_from->K;
    uint cost = 0;
    for (uint i = 0; i < N; ++i) {
      // no cost only when staying at the final goal after the sequence
      // (K checked last: it rarely matters and this loop is hot in rewrite)
      if (C1[i] != ins->goals[i] || C2[i] != ins->goals[i] ||
          K1[i] != K_last[i]) {
        cost += 1;
      }
    }
    return cost;
  }

  // default: makespan
  return 1;
}

uint Planner::get_edge_cost(HNode* H_from, HNode* H_to)
{
  return get_edge_cost(H_from, H_to->C);
}

uint Planner::get_h_value(const Config& C, const std::vector<uint>& K,
                          const Windows& W)
{
  // time_to_go is D.h without STN, plus the waits for the windows with STN
  uint cost = 0;
  if (objective == OBJ_MAKESPAN) {
    for (auto i = 0; i < N; ++i)
      cost = std::max(cost, time_to_go(*ins, D, K, W, i, C[i]));
  } else if (objective == OBJ_SUM_OF_LOSS) {
    for (auto i = 0; i < N; ++i) cost += time_to_go(*ins, D, K, W, i, C[i]);
  }
  return cost;
}

bool Planner::meets_deadlines(const Config& C, const std::vector<uint>& K,
                              const Windows& W)
{
  if (ins->stn == nullptr || !ins->stn->has_deadline) return true;
  for (uint i = 0; i < N; ++i)
    if (time_to_go(*ins, D, K, W, i, C[i]) >= (uint)TemporalNetwork::INF)
      return false;
  return true;
}

bool Planner::is_goal(const Config& C, const std::vector<uint>& K) const
{
  for (uint i = 0; i < N; ++i)
    if (!ins->intermediates_done(K, i)) return false;
  return is_same_config(C, ins->goals);
}

void Planner::expand_lowlevel_tree(HNode* H, const LNodePtr& L)
{
  if (L->depth >= N) return;
  const auto i = H->order[L->depth];
  auto C = H->C[i]->neighbor;
  C.push_back(H->C[i]);
  // vertices reserved for other agents
  if (!ins->reserved.empty())
    C.erase(std::remove_if(C.begin(), C.end(),
                           [&](Vertex* v) { return ins->forbidden(i, v); }),
            C.end());
  // randomize
  if (MT != nullptr) std::shuffle(C.begin(), C.end(), *MT);
  // insert
  for (auto v : C) H->search_tree.push(std::make_shared<LNode>(L, i, v));
}

bool Planner::get_new_config(HNode* H, const LNodePtr& L)
{
  // PIBT ranks moves by the distance to current waypoints
  K_cur = &H->K;
  for (uint i = 0; i < N; ++i) slot_cur[i] = D.wp_slot[i][H->K[i]];

  // setup cache
  for (auto a : A) {
    // clear previous cache
    if (a->v_now != nullptr && occupied_now[a->v_now->id] == a) {
      occupied_now[a->v_now->id] = nullptr;
    }
    if (a->v_next != nullptr) {
      occupied_next[a->v_next->id] = nullptr;
      a->v_next = nullptr;
    }

    // set occupied now
    a->v_now = H->C[a->id];
    occupied_now[a->v_now->id] = a;
  }

  // collect constraints, root first (the root has no constraint)
  L_chain.clear();
  for (auto n = L.get(); n->parent != nullptr; n = n->parent.get())
    L_chain.push_back(n);
  std::reverse(L_chain.begin(), L_chain.end());

  // add constraints
  for (auto n : L_chain) {
    const auto i = n->who;         // agent
    const auto l = n->where->id;   // loc

    // check vertex collision
    if (occupied_next[l] != nullptr) return false;
    // check swap collision
    auto l_pre = H->C[i]->id;
    if (occupied_next[l_pre] != nullptr && occupied_now[l] != nullptr &&
        occupied_next[l_pre]->id == occupied_now[l]->id)
      return false;

    // set occupied_next
    A[i]->v_next = n->where;
    occupied_next[l] = A[i];
  }

  // perform PIBT
  for (auto k : H->order) {
    auto a = A[k];
    if (a->v_next == nullptr && !funcPIBT(a)) return false;  // planning failure
  }
  return true;
}

bool Planner::funcPIBT(Agent* ai)
{
  const auto i = ai->id;
  const auto K = ai->v_now->neighbor.size();

  // get candidates for next locations
  for (auto k = 0; k < K; ++k) {
    auto u = ai->v_now->neighbor[k];
    C_next[i][k] = u;
    if (MT != nullptr)
      tie_breakers[u->id] = get_random_float(MT);  // set tie-breaker
  }
  C_next[i][K] = ai->v_now;

  // sort
  std::sort(C_next[i].begin(), C_next[i].begin() + K + 1,
            [&](Vertex* const v, Vertex* const u) {
              return dist_cur(i, v) + tie_breakers[v->id] <
                     dist_cur(i, u) + tie_breakers[u->id];
            });

  Agent* swap_agent = swap_possible_and_required(ai);
  if (swap_agent != nullptr)
    std::reverse(C_next[i].begin(), C_next[i].begin() + K + 1);

  // main operation
  for (auto k = 0; k < K + 1; ++k) {
    auto u = C_next[i][k];

    // avoid vertex conflicts
    if (occupied_next[u->id] != nullptr) continue;
    // vertices reserved for other agents
    if (ins->forbidden(i, u)) continue;

    auto& ak = occupied_now[u->id];

    // avoid swap conflicts
    if (ak != nullptr && ak->v_next == ai->v_now) continue;

    // reserve next location
    occupied_next[u->id] = ai;
    ai->v_next = u;

    // priority inheritance
    if (ak != nullptr && ak != ai && ak->v_next == nullptr && !funcPIBT(ak))
      continue;

    // success to plan next one step
    // pull swap_agent when applicable
    if (k == 0 && swap_agent != nullptr && swap_agent->v_next == nullptr &&
        occupied_next[ai->v_now->id] == nullptr &&
        !ins->forbidden(swap_agent->id, ai->v_now)) {
      swap_agent->v_next = ai->v_now;
      occupied_next[swap_agent->v_next->id] = swap_agent;
    }
    return true;
  }

  // failed to secure node
  occupied_next[ai->v_now->id] = ai;
  ai->v_next = ai->v_now;
  return false;
}

Agent* Planner::swap_possible_and_required(Agent* ai)
{
  const auto i = ai->id;
  // ai wanna stay at v_now -> no need to swap
  if (C_next[i][0] == ai->v_now) return nullptr;

  // usual swap situation, c.f., case-a, b
  auto aj = occupied_now[C_next[i][0]->id];
  if (aj != nullptr && aj->v_next == nullptr &&
      is_swap_required(ai->id, aj->id, ai->v_now, aj->v_now) &&
      is_swap_possible(aj->v_now, ai->v_now)) {
    return aj;
  }

  // for clear operation, c.f., case-c
  for (auto u : ai->v_now->neighbor) {
    auto ak = occupied_now[u->id];
    if (ak == nullptr || C_next[i][0] == ak->v_now) continue;
    if (is_swap_required(ak->id, ai->id, ai->v_now, C_next[i][0]) &&
        is_swap_possible(C_next[i][0], ai->v_now)) {
      return ak;
    }
  }

  return nullptr;
}

// simulate whether the swap is required
bool Planner::is_swap_required(const uint pusher, const uint puller,
                               Vertex* v_pusher_origin, Vertex* v_puller_origin)
{
  auto v_pusher = v_pusher_origin;
  auto v_puller = v_puller_origin;
  Vertex* tmp = nullptr;
  while (dist_cur(pusher, v_puller) < dist_cur(pusher, v_pusher)) {
    auto n = v_puller->neighbor.size();
    // remove agents who need not to move
    for (auto u : v_puller->neighbor) {
      auto a = occupied_now[u->id];
      if (u == v_pusher ||
          (u->neighbor.size() == 1 && a != nullptr && is_done_at(a->id, u))) {
        --n;
      } else {
        tmp = u;
      }
    }
    if (n >= 2) return false;  // able to swap
    if (n <= 0) break;
    v_pusher = v_puller;
    v_puller = tmp;
  }

  // judge based on distance
  return (dist_cur(puller, v_pusher) < dist_cur(puller, v_puller)) &&
         (dist_cur(pusher, v_pusher) == 0 ||
          dist_cur(pusher, v_puller) < dist_cur(pusher, v_pusher));
}

// simulate whether the swap is possible
bool Planner::is_swap_possible(Vertex* v_pusher_origin, Vertex* v_puller_origin)
{
  auto v_pusher = v_pusher_origin;
  auto v_puller = v_puller_origin;
  Vertex* tmp = nullptr;
  while (v_puller != v_pusher_origin) {  // avoid loop
    auto n = v_puller->neighbor.size();  // count #(possible locations) to pull
    for (auto u : v_puller->neighbor) {
      auto a = occupied_now[u->id];
      if (u == v_pusher ||
          (u->neighbor.size() == 1 && a != nullptr && is_done_at(a->id, u))) {
        --n;      // pull-impossible with u
      } else {
        tmp = u;  // pull-possible with u
      }
    }
    if (n >= 2) return true;  // able to swap
    if (n <= 0) return false;
    v_pusher = v_puller;
    v_puller = tmp;
  }
  return false;
}

std::ostream& operator<<(std::ostream& os, const Objective obj)
{
  if (obj == OBJ_NONE) {
    os << "none";
  } else if (obj == OBJ_MAKESPAN) {
    os << "makespan";
  } else if (obj == OBJ_SUM_OF_LOSS) {
    os << "sum_of_loss";
  }
  return os;
}

std::ostream& operator<<(std::ostream& os, const PriorityReset reset)
{
  if (reset == WP_RESET_FINAL) {
    os << "final";
  } else if (reset == WP_RESET_EACH) {
    os << "each";
  }
  return os;
}
