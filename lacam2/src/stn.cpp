#include "../include/stn.hpp"

#include "../include/dist_table.hpp"

TemporalNetwork::TemporalNetwork(std::istream& is, const Instance& ins)
    : E(0), has_deadline(false), valid(false)
{
  load(is, ins);
}

TemporalNetwork::TemporalNetwork(const std::string& filename,
                                 const Instance& ins)
    : E(0), has_deadline(false), valid(false)
{
  std::ifstream file(filename);
  if (!file) {
    fail(filename + " is not found");
    return;
  }
  load(file, ins);
}

bool TemporalNetwork::fail(const std::string& msg)
{
  error = msg;
  valid = false;
  return false;
}

void TemporalNetwork::load(std::istream& is, const Instance& ins)
{
  // tokens, without comments
  auto tokens = std::vector<std::string>();
  std::string line;
  while (std::getline(is, line)) {
    const auto c = line.find('#');
    if (c != std::string::npos) line.resize(c);
    std::istringstream iss(line);
    std::string tok;
    while (iss >> tok) tokens.push_back(tok);
  }
  size_t pos = 0;
  auto next = [&](std::string& tok) {
    if (pos >= tokens.size()) return false;
    tok = tokens[pos++];
    return true;
  };
  auto next_int = [&](long long& x) {
    std::string tok;
    if (!next(tok)) return false;
    try {
      size_t used;
      x = std::stoll(tok, &used);
      return used == tok.size();
    } catch (...) {
      return false;
    }
  };

  // timepoints
  std::string tok;
  long long T, M;
  if (!next(tok) || tok != "timepoints" || !next_int(T) || T < 0) {
    fail("expected 'timepoints T'");
    return;
  }
  const auto N = ins.N;
  event_of.assign(N, std::vector<int>());
  for (uint i = 0; i < N; ++i) event_of[i].assign(ins.m(i), -1);
  auto tp_agent = std::vector<long long>(T + 1, -1);
  auto tp_k = std::vector<long long>(T + 1, 0);
  for (long long t = 1; t <= T; ++t) {
    if (!next_int(tp_agent[t]) || !next_int(tp_k[t])) {
      fail("timepoint " + std::to_string(t) + ": expected '<agent> <k>'");
      return;
    }
    const auto a = tp_agent[t], k = tp_k[t];
    if (a == -1) continue;  // free timepoint
    if (a < 0 || a >= (long long)N) {
      fail("timepoint " + std::to_string(t) + ": invalid agent");
      return;
    }
    // only intermediate waypoints have events
    if (k < 0 || k + 1 >= (long long)ins.m(a)) {
      fail("timepoint " + std::to_string(t) + ": agent " + std::to_string(a) +
           " has no intermediate waypoint " + std::to_string(k));
      return;
    }
    if (event_of[a][k] != -1) {
      fail("timepoint " + std::to_string(t) + ": duplicated event");
      return;
    }
    event_of[a][k] = t;  // temporarily, timepoint id
  }

  // full network, as a distance graph
  using ll = long long;
  const ll INF_LL = (ll)1 << 50;
  const auto n = T + 1;
  auto F = std::vector<std::vector<ll> >(n, std::vector<ll>(n, INF_LL));
  for (ll a = 0; a < n; ++a) F[a][a] = 0;
  auto add = [&](ll a, ll b, ll lb, ll ub) {  // lb <= t_b - t_a <= ub
    F[a][b] = std::min(F[a][b], ub);
    F[b][a] = std::min(F[b][a], -lb);
  };
  if (!next(tok) || tok != "constraints" || !next_int(M) || M < 0) {
    fail("expected 'constraints M'");
    return;
  }
  for (ll c = 0; c < M; ++c) {
    ll a, b, bound[2];
    if (!next_int(a) || !next_int(b) || a < 0 || b < 0 || a >= n || b >= n) {
      fail("constraint " + std::to_string(c) + ": invalid timepoints");
      return;
    }
    for (int j = 0; j < 2; ++j) {
      if (pos < tokens.size() && tokens[pos] == "-") {
        ++pos;
        bound[j] = j == 0 ? -INF_LL : INF_LL;
      } else if (!next_int(bound[j])) {
        fail("constraint " + std::to_string(c) + ": invalid bound");
        return;
      }
    }
    add(a, b, bound[0], bound[1]);
  }
  if (pos != tokens.size()) {
    fail("unexpected token: " + tokens[pos]);
    return;
  }
  // implied by the semantics: events after the origin, in sequence order
  for (uint i = 0; i < N; ++i) {
    ll prev = 0;
    for (auto t : event_of[i]) {
      if (t < 0) continue;
      add(prev, t, 0, INF_LL);
      prev = t;
    }
  }

  // minimal network
  for (ll k = 0; k < n; ++k)
    for (ll a = 0; a < n; ++a) {
      if (F[a][k] >= INF_LL) continue;
      for (ll b = 0; b < n; ++b) {
        if (F[k][b] >= INF_LL) continue;
        F[a][b] = std::min(F[a][b], F[a][k] + F[k][b]);
      }
    }
  for (ll a = 0; a < n; ++a)
    if (F[a][a] < 0) {
      fail("inconsistent STN (negative cycle)");
      return;
    }

  // projection on the origin and the events, sorted by (agent, k)
  auto tp_of_event = std::vector<ll>({0});
  agent.assign(1, N);
  index.assign(1, 0);
  for (uint i = 0; i < N; ++i)
    for (uint k = 0; k < event_of[i].size(); ++k) {
      if (event_of[i][k] < 0) continue;
      tp_of_event.push_back(event_of[i][k]);
      event_of[i][k] = agent.size();
      agent.push_back(i);
      index.push_back(k);
    }
  E = agent.size();
  D.assign(E, std::vector<int>(E, INF));
  for (uint a = 0; a < E; ++a)
    for (uint b = 0; b < E; ++b) {
      const auto d = F[tp_of_event[a]][tp_of_event[b]];
      if (d >= INF_LL / 2) continue;
      if (std::abs(d) >= INF) {
        fail("bound out of range");
        return;
      }
      D[a][b] = d;
    }

  // only lower bounds between events. When e fires, x gets the deadline
  // D[e][x] (relative to now): if D[e][x] <= 0, x must fire before e and is
  // a predecessor; otherwise the deadline must be implied by the absolute
  // window of x, D[e][x] = D[e][0] + D[0][x], so that firing e as early as
  // possible is never worse
  auto name = [&](uint e) {
    return "(" + std::to_string(agent[e]) + "," + std::to_string(index[e]) +
           ")";
  };
  for (uint a = 1; a < E; ++a)
    for (uint b = 1; b < E; ++b) {
      if (a == b || D[a][b] >= INF) continue;
      if (D[a][b] <= 0) {
        // both <= 0: simultaneous events
        if (D[b][a] <= 0 && agent[a] != agent[b]) {
          fail("events " + name(a) + " and " + name(b) +
               " must be simultaneous: not supported");
          return;
        }
        continue;
      }
      if (D[0][b] >= INF || D[a][b] < D[a][0] + D[0][b]) {
        fail("upper bound between events " + name(a) + " and " + name(b) +
             ": only lower bounds and absolute windows are supported");
        return;
      }
    }

  preds.assign(E, std::vector<uint>());
  for (uint e = 1; e < E; ++e) {
    if (D[0][e] < INF) has_deadline = true;
    for (uint p = 1; p < E; ++p)
      if (agent[p] != agent[e] && D[e][p] <= 0) preds[e].push_back(p);
  }
  valid = true;
}

uint hash_windows(const Windows& W, uint hash)
{
  for (auto& w : W) {
    hash ^= w.e + 0x9e3779b9 + (hash << 6) + (hash >> 2);
    hash ^= (uint)w.lo + 0x9e3779b9 + (hash << 6) + (hash >> 2);
    hash ^= (uint)w.hi + 0x9e3779b9 + (hash << 6) + (hash >> 2);
  }
  return hash;
}

// event e has fired, i.e., its agent has passed its waypoint
static inline bool executed(const TemporalNetwork& S,
                            const std::vector<uint>& K, uint e)
{
  return K[S.agent[e]] > S.index[e];
}

static inline bool enabled(const TemporalNetwork& S,
                           const std::vector<uint>& K, uint e)
{
  for (auto p : S.preds[e])
    if (!executed(S, K, p)) return false;
  return true;
}

bool advance_state(const Instance& ins, const Config& C, std::vector<uint>& K,
                   Windows& W, const bool shift)
{
  if (ins.stn == nullptr) {
    advance_K(ins, C, K);
    return true;
  }
  const auto& S = *ins.stn;
  const auto INF = TemporalNetwork::INF;

  // dense windows, after one time step
  thread_local std::vector<int> lo, hi;
  lo.assign(S.E, 0);
  hi.assign(S.E, INF);
  const int dt = shift ? 1 : 0;
  for (auto& w : W) {
    lo[w.e] = std::max(0, w.lo - dt);
    hi[w.e] = w.hi >= INF ? INF : w.hi - dt;
  }

  // fire, until no event can fire
  bool fired = true;
  while (fired) {
    fired = false;
    for (uint i = 0; i < ins.N; ++i) {
      while (K[i] + 1 < ins.m(i) && C[i] == ins.waypoints[i][K[i]]) {
        const auto e = S.event_of[i][K[i]];
        if (e > 0) {
          if (lo[e] > 0 || hi[e] < 0 || !enabled(S, K, e)) break;
          // tighten the windows of the others, e fires now (relative 0)
          const auto& De = S.D[e];
          for (uint x = 1; x < S.E; ++x) {
            if ((int)x == e) continue;
            if (S.D[x][e] < INF) lo[x] = std::max(lo[x], -S.D[x][e]);
            if (De[x] < INF) hi[x] = std::min(hi[x], De[x]);
          }
        }
        ++K[i];
        fired = true;
      }
    }
  }

  // sparse windows of unexecuted events, dead state check
  W.clear();
  bool alive = true;
  for (uint x = 1; x < S.E; ++x) {
    if (executed(S, K, x)) continue;
    const auto l = std::max(lo[x], 0);
    if (hi[x] < l) alive = false;
    if (l > 0 || hi[x] < INF) W.push_back({x, l, hi[x]});
  }
  return alive;
}

bool initial_state(const Instance& ins, std::vector<uint>& K, Windows& W)
{
  K.assign(ins.N, 0);
  W.clear();
  if (ins.stn == nullptr) {
    advance_K(ins, ins.starts, K);
    return true;
  }
  // absolute windows, from the origin
  const auto& S = *ins.stn;
  for (uint e = 1; e < S.E; ++e) {
    const auto l = std::max(0, -S.D[e][0]);
    const auto h = S.D[0][e];
    if (l > 0 || h < TemporalNetwork::INF) W.push_back({e, l, h});
  }
  return advance_state(ins, ins.starts, K, W, false);
}

static inline const Window* find_window(const Windows& W, uint e)
{
  auto it = std::lower_bound(W.begin(), W.end(), e,
                             [](const Window& w, uint e) { return w.e < e; });
  return (it != W.end() && it->e == e) ? &*it : nullptr;
}

int event_release(const Instance& ins, const std::vector<uint>& K,
                  const Windows& W, uint i)
{
  if (ins.stn == nullptr || K[i] + 1 >= ins.m(i)) return 0;
  const auto& S = *ins.stn;
  const auto e = S.event_of[i][K[i]];
  if (e <= 0) return 0;
  if (!enabled(S, K, e)) return TemporalNetwork::INF;
  const auto w = find_window(W, e);
  return w == nullptr ? 0 : w->lo;
}

// lower bound on the firing time of each remaining event of agent i (tau),
// from v; with prune, windows whose lo is already implied by tau are dropped
// (lo can never bind: the agent cannot fire earlier than tau anyway).
// Returns the bound on the completion time, INF if a deadline is missed.
static uint lookahead(const Instance& ins, DistTable& D,
                      const std::vector<uint>& K, Windows& W, uint i,
                      Vertex* v, const bool prune,
                      std::vector<long long>* tau_of = nullptr)
{
  const auto& S = *ins.stn;
  const long long INF = TemporalNetwork::INF;
  const auto m = ins.m(i);

  thread_local std::vector<std::pair<uint, long long> > fired;  // (e, tau)
  fired.clear();
  long long tau = 0;
  auto pos = v;
  bool dead = false;
  for (uint k = K[i]; k + 1 < m; ++k) {
    tau += D.get_wp(i, k, pos);
    const auto e = S.event_of[i][k];
    if (e > 0) {
      // lower bounds from the earlier events of the same agent
      for (auto& [p, t] : fired)
        if (S.D[e][p] < INF) tau = std::max(tau, t - S.D[e][p]);
      auto it = std::lower_bound(
          W.begin(), W.end(), (uint)e,
          [](const Window& w, uint e) { return w.e < e; });
      if (it != W.end() && it->e == (uint)e) {
        if (prune && it->lo <= tau) it->lo = 0;  // implied, see above
        tau = std::max(tau, (long long)it->lo);
        if (tau > it->hi) dead = true;
      }
      fired.push_back({(uint)e, tau});
      if (tau_of != nullptr) (*tau_of)[e] = tau;
    }
    pos = ins.waypoints[i][k];
  }
  return dead ? INF : tau + D.get_wp(i, m - 1, pos);
}

uint time_to_go(const Instance& ins, DistTable& D, const std::vector<uint>& K,
                const Windows& W, uint i, Vertex* v)
{
  if (ins.stn == nullptr) return D.h(i, K[i], v);
  return lookahead(ins, D, K, const_cast<Windows&>(W), i, v, false);
}

void prune_windows(const Instance& ins, DistTable& D, const Config& C,
                   const std::vector<uint>& K, Windows& W)
{
  if (ins.stn == nullptr || W.empty()) return;
  const auto& S = *ins.stn;
  // 1. implied by the travel of the agent itself
  thread_local std::vector<long long> tau_of;
  tau_of.assign(S.E, 0);
  for (uint i = 0; i < ins.N; ++i)
    lookahead(ins, D, K, W, i, C[i], true, &tau_of);
  // 2. implied by a predecessor p that has not fired: x cannot fire before p,
  // and when p fires (not before tau_of[p]) x gets lo >= -D[x][p] again
  for (auto& w : W) {
    if (w.lo == 0) continue;
    for (auto p : S.preds[w.e]) {
      if (!executed(S, K, p) && tau_of[p] - S.D[w.e][p] >= w.lo) {
        w.lo = 0;
        break;
      }
    }
  }
  W.erase(std::remove_if(W.begin(), W.end(),
                         [](const Window& w) {
                           return w.lo == 0 && w.hi >= TemporalNetwork::INF;
                         }),
          W.end());
}
