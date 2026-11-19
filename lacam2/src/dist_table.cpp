#include "../include/dist_table.hpp"

DistTable::DistTable(const Instance& ins) : V_size(ins.G.V.size())
{
  setup(&ins);
}

DistTable::DistTable(const Instance* ins) : V_size(ins->G.V.size())
{
  setup(ins);
}

void DistTable::setup(const Instance* ins)
{
  // register distinct targets
  auto slot_of_vertex = std::vector<int>(V_size, -1);
  for (size_t i = 0; i < ins->N; ++i) {
    wp_slot.push_back(std::vector<uint>());
    for (auto w : ins->waypoints[i]) {
      if (slot_of_vertex[w->id] < 0) {
        slot_of_vertex[w->id] = targets.size();
        targets.push_back(w);
        table.push_back(std::vector<uint>());  // lazy allocation
        OPEN.push_back(std::queue<Vertex*>());
      }
      wp_slot[i].push_back(slot_of_vertex[w->id]);
    }
  }

  // remaining distances along each sequence
  for (size_t i = 0; i < ins->N; ++i) {
    const auto& W = ins->waypoints[i];
    const auto m = W.size();
    rem.push_back(std::vector<uint>(m + 1, 0));
    for (int k = (int)m - 2; k >= 0; --k) {
      rem[i][k] = rem[i][k + 1] + get_wp(i, k + 1, W[k]);
    }
  }
}

uint DistTable::get_slot(uint s, uint v_id)
{
  auto& T = table[s];
  if (T.empty()) {
    T.assign(V_size, V_size);
    T[targets[s]->id] = 0;
    OPEN[s].push(targets[s]);
  }
  if (T[v_id] < V_size) return T[v_id];

  /*
   * BFS with lazy evaluation
   * c.f., Reverse Resumable A*
   * https://www.aaai.org/Papers/AIIDE/2005/AIIDE05-020.pdf
   *
   * sidenote:
   * tested RRA* but lazy BFS was much better in performance
   */

  auto& Q = OPEN[s];
  while (!Q.empty()) {
    auto n = Q.front();  // copy, front() is invalidated by pop()
    Q.pop();
    const int d_n = T[n->id];
    for (auto&& m : n->neighbor) {
      const int d_m = T[m->id];
      if (d_n + 1 >= d_m) continue;
      T[m->id] = d_n + 1;
      Q.push(m);
    }
    if (n->id == v_id) return d_n;
  }
  return V_size;
}

uint DistTable::get(uint i, uint v_id)
{
  return get_slot(wp_slot[i].back(), v_id);
}

uint DistTable::get(uint i, Vertex* v) { return get(i, v->id); }
