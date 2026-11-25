/*
 * lacam-star
 */

#pragma once

#include "dist_table.hpp"
#include "graph.hpp"
#include "instance.hpp"
#include "stn.hpp"
#include "utils.hpp"

// objective function
enum Objective { OBJ_NONE, OBJ_MAKESPAN, OBJ_SUM_OF_LOSS };
std::ostream& operator<<(std::ostream& os, const Objective objective);

// when the PIBT priority of an agent with waypoints is reset
enum PriorityReset {
  WP_RESET_FINAL,  // only when the whole sequence is completed
  WP_RESET_EACH    // also when reaching each intermediate waypoint
};
std::ostream& operator<<(std::ostream& os, const PriorityReset reset);

// how the PIBT order of the agents is set
enum PriorityMode {
  PRIO_DIST,  // distance to go, elevated by one per step (PIBT)
  PRIO_SLACK,  // slack w.r.t. the deadlines from a target makespan, see HNode
  // as PRIO_SLACK, the earliest arrival also waits for the release of the
  // next event (window lo), when known
  PRIO_SLACK_RELEASE
};
std::ostream& operator<<(std::ostream& os, const PriorityMode mode);

// hash of a high-level search state: locations, waypoint progress and
// time windows
uint hash_state(const Config& C, const std::vector<uint>& K,
                const Windows& W = Windows());

// PIBT agent
struct Agent {
  const uint id;
  Vertex* v_now;   // current location
  Vertex* v_next;  // next location
  Agent(uint _id) : id(_id), v_now(nullptr), v_next(nullptr) {}
};
using Agents = std::vector<Agent*>;

// low-level node: constraint "agent who goes to where", plus the constraints
// of its ancestors; ancestors are shared, so the memory per node is constant
// instead of O(depth)
struct LNode;
using LNodePtr = std::shared_ptr<LNode>;
struct LNode {
  const LNodePtr parent;
  const uint who;
  Vertex* const where;
  const uint depth;
  LNode(LNodePtr _parent = nullptr, uint i = 0,
        Vertex* v = nullptr);  // who and where
};

// high-level node
struct HNode {
  static uint HNODE_CNT;  // count #(high-level node)
  const Config C;
  const std::vector<uint> K;  // waypoint progress, see advance_K
  const Windows W;            // time windows, see advance_state

  // tree
  HNode* parent;
  std::set<HNode*> neighbor;
  const uint t;  // time step when created, i.e., depth of the first parent

  // costs
  uint g;        // g-value (might be updated)
  const uint h;  // h-value
  uint f;        // g + h (might be updated)

  // for low-level search
  std::vector<float> priorities;
  std::vector<uint> order;
  std::queue<LNodePtr> search_tree;

  // deadlines (waypoint_deadlines) != nullptr: slack-based order, agents by
  // increasing slack(i) = dl[i][K[i]] - (t + dist(C[i], next waypoint)),
  // ties by the order of the parent, agents done last; reset is ignored.
  // With release, the distance is max(distance, event_release) unless a
  // predecessor of another agent has not fired (release unknown).
  // Otherwise distance-based PIBT priorities.
  HNode(const Config& _C, const std::vector<uint>& _K, const Windows& _W,
        const Instance* ins, DistTable& D, HNode* _parent, const uint _g,
        const uint _h, const PriorityReset reset = WP_RESET_EACH,
        const std::vector<std::vector<int> >* deadlines = nullptr,
        const bool release = false);
  ~HNode();
};
using HNodes = std::vector<HNode*>;

// explored high-level nodes, keyed by (C, K, W)
// the key is not copied: candidates with the same hash are compared with the
// node's own C and K, which halves the memory per node for large N
struct Explored {
  std::unordered_multimap<uint, HNode*> table;  // hash -> node
  HNode* find(const Config& C, const std::vector<uint>& K, const Windows& W,
              uint hash) const;
  void insert(HNode* H, uint hash) { table.emplace(hash, H); }
  size_t size() const { return table.size(); }
};

struct Planner {
  const Instance* ins;
  const Deadline* deadline;
  std::mt19937* MT;
  const int verbose;

  // hyper parameters
  const Objective objective;
  const float RESTART_RATE;  // random restart
  const PriorityReset WP_RESET;
  const PriorityMode PRIORITY;

  // solver utils
  const uint N;       // number of agents
  const uint V_size;  // number o vertices
  DistTable D;
  uint loop_cnt;      // auxiliary
  // PRIO_SLACK: target makespan, max of the completion lower bounds, and
  // the deadline of each waypoint w.r.t. it (computed once)
  int target_makespan;
  std::vector<std::vector<int> > wp_deadline;
  // anytime profile: "elapsed_ms:cost:makespan" at the first solution and at
  // each improvement, reported in additional_info as "anytime=..."
  std::string anytime_log;
  void log_anytime(HNode* H_goal);

  // used in PIBT
  std::vector<std::array<Vertex*, 5> > C_next;  // next locations, used in PIBT
  std::vector<float> tie_breakers;              // random values, used in PIBT
  Agents A;
  Agents occupied_now;                          // for quick collision checking
  Agents occupied_next;                         // for quick collision checking
  const std::vector<uint>* K_cur;  // waypoint progress of the expanded node
  std::vector<uint> slot_cur;      // DistTable slot of the current waypoints
  const std::vector<uint> K_last;  // m_i - 1, i.e., heading to final goal

  Planner(const Instance* _ins, const Deadline* _deadline, std::mt19937* _MT,
          const int _verbose = 0,
          // other parameters
          const Objective _objective = OBJ_NONE,
          const float _restart_rate = 0.001,
          const PriorityReset _wp_reset = WP_RESET_EACH,
          const PriorityMode _priority = PRIO_DIST);
  ~Planner();
  Solution solve(std::string& additional_info);
  void expand_lowlevel_tree(HNode* H, const LNodePtr& L);
  void rewrite(HNode* H_from, HNode* T, HNode* H_goal,
               std::stack<HNode*>& OPEN);
  uint get_edge_cost(HNode* H_from, const Config& C2);
  uint get_edge_cost(HNode* H_from, HNode* H_to);
  uint get_h_value(const Config& C, const std::vector<uint>& K,
                   const Windows& W);
  // with STN deadlines: every agent can still meet them (lookahead)
  bool meets_deadlines(const Config& C, const std::vector<uint>& K,
                       const Windows& W);
  bool is_goal(const Config& C, const std::vector<uint>& K) const;
  // agent i completed its sequence and stays at its final goal v
  inline bool is_done_at(uint i, Vertex* v) const
  {
    return (*K_cur)[i] == K_last[i] && ins->goals[i] == v;
  }
  // distance to the current waypoint, used to rank moves in PIBT
  inline uint dist_cur(uint i, Vertex* v)
  {
    return D.get_slot(slot_cur[i], v->id);
  }
  bool get_new_config(HNode* H, const LNodePtr& L);
  std::vector<LNode*> L_chain;  // constraints of a low-level node, root first
  bool funcPIBT(Agent* ai);

  // swap operation
  Agent* swap_possible_and_required(Agent* ai);
  bool is_swap_required(const uint pusher, const uint puller,
                        Vertex* v_pusher_origin, Vertex* v_puller_origin);
  bool is_swap_possible(Vertex* v_pusher_origin, Vertex* v_puller_origin);

  // utilities
  template <typename... Body>
  void solver_info(const int level, Body&&... body)
  {
    if (verbose < level) return;
    std::cout << "elapsed:" << std::setw(6) << elapsed_ms(deadline) << "ms"
              << "  loop_cnt:" << std::setw(8) << loop_cnt
              << "  node_cnt:" << std::setw(8) << HNode::HNODE_CNT << "\t";
    info(level, verbose, (body)...);
  }
};
