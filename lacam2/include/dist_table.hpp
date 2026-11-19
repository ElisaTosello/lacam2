/*
 * distance table with lazy evaluation, using BFS
 *
 * One table per distinct target vertex (waypoint), shared among agents.
 */
#pragma once

#include "graph.hpp"
#include "instance.hpp"
#include "utils.hpp"

struct DistTable {
  const uint V_size;  // number of vertices
  // distance table, index: slot (distinct target) & vertex-id,
  // allocated on first use
  std::vector<std::vector<uint> > table;
  std::vector<std::queue<Vertex*> > OPEN;  // search queue
  std::vector<Vertex*> targets;            // slot -> target vertex
  // wp_slot[i][j]: slot of the j-th waypoint of agent i
  std::vector<std::vector<uint> > wp_slot;
  // rem[i][k]: sum_{j >= k} dist(w_i^j, w_i^{j+1}), size m_i + 1
  std::vector<std::vector<uint> > rem;

  uint get_slot(uint s, uint v_id);  // slot, vertex-id
  uint get(uint i, uint v_id);       // agent, vertex-id; to final goal
  uint get(uint i, Vertex* v);       // agent, vertex; to final goal
  // distance to the k-th waypoint of agent i, k is clamped to m_i - 1
  // (K never exceeds m_i - 1 in the planner, clamping is just defensive)
  inline uint get_wp(uint i, uint k, Vertex* v)
  {
    const auto& S = wp_slot[i];
    return get_slot(S[k < S.size() ? k : S.size() - 1], v->id);
  }
  // remaining distance along the sequence, from v with next waypoint k
  inline uint h(uint i, uint k, Vertex* v) { return get_wp(i, k, v) + rem[i][k]; }

  DistTable(const Instance& ins);
  DistTable(const Instance* ins);

  void setup(const Instance* ins);  // initialization
};
