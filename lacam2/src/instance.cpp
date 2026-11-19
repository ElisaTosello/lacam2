#include "../include/instance.hpp"

Instance::Instance(const std::string& map_filename,
                   const std::vector<uint>& start_indexes,
                   const std::vector<uint>& goal_indexes)
    : G(map_filename),
      starts(Config()),
      goals(Config()),
      N(start_indexes.size())
{
  for (auto k : start_indexes) starts.push_back(G.U[k]);
  for (auto k : goal_indexes) goals.push_back(G.U[k]);
  for (auto g : goals) waypoints.push_back({g});
}

Instance::Instance(const std::string& map_filename,
                   const std::vector<uint>& start_indexes,
                   const std::vector<std::vector<uint> >& waypoint_indexes)
    : G(map_filename),
      starts(Config()),
      goals(Config()),
      N(start_indexes.size())
{
  for (auto k : start_indexes) starts.push_back(G.U[k]);
  for (auto& seq : waypoint_indexes) {
    waypoints.push_back(Vertices());
    for (auto k : seq) waypoints.back().push_back(G.U[k]);
    if (!seq.empty()) goals.push_back(waypoints.back().back());
  }
}

// for load instance
static const std::regex r_instance =
    std::regex(R"(\d+\t.+\.map\t\d+\t\d+\t(\d+)\t(\d+)\t(\d+)\t(\d+)\t.+)");

Instance::Instance(const std::string& scen_filename,
                   const std::string& map_filename, const uint _N)
    : G(Graph(map_filename)), starts(Config()), goals(Config()), N(_N)
{
  // load start-goal pairs
  std::ifstream file(scen_filename);
  if (!file) {
    info(0, 0, scen_filename, " is not found");
    return;
  }
  std::string line;
  std::smatch results;

  while (getline(file, line)) {
    // for CRLF coding
    if (*(line.end() - 1) == 0x0d) line.pop_back();

    if (std::regex_match(line, results, r_instance)) {
      uint x_s = std::stoi(results[1].str());
      uint y_s = std::stoi(results[2].str());
      uint x_g = std::stoi(results[3].str());
      uint y_g = std::stoi(results[4].str());
      if (x_s < 0 || G.width <= x_s || x_g < 0 || G.width <= x_g) break;
      if (y_s < 0 || G.height <= y_s || y_g < 0 || G.height <= y_g) break;
      auto s = G.U[G.width * y_s + x_s];
      auto g = G.U[G.width * y_g + x_g];
      if (s == nullptr || g == nullptr) break;
      starts.push_back(s);
      goals.push_back(g);
    }

    if (starts.size() == N) break;
  }
  for (auto g : goals) waypoints.push_back({g});
}

Instance::Instance(const std::string& map_filename, std::mt19937* MT,
                   const uint _N)
    : G(Graph(map_filename)), starts(Config()), goals(Config()), N(_N)
{
  // random assignment
  const auto V_size = G.size();

  // set starts
  auto s_indexes = std::vector<uint>(V_size);
  std::iota(s_indexes.begin(), s_indexes.end(), 0);
  std::shuffle(s_indexes.begin(), s_indexes.end(), *MT);
  size_t i = 0;
  while (true) {
    if (i >= V_size) return;
    starts.push_back(G.V[s_indexes[i]]);
    if (starts.size() == N) break;
    ++i;
  }

  // set goals
  auto g_indexes = std::vector<uint>(V_size);
  std::iota(g_indexes.begin(), g_indexes.end(), 0);
  std::shuffle(g_indexes.begin(), g_indexes.end(), *MT);
  size_t j = 0;
  while (true) {
    if (j >= V_size) return;
    goals.push_back(G.V[g_indexes[j]]);
    if (goals.size() == N) break;
    ++j;
  }
  for (auto g : goals) waypoints.push_back({g});
}

static const std::regex r_wp_comment = std::regex(R"(\s*(#.*)?)");

Instance::Instance(const std::string& map_filename,
                   const std::string& wp_filename, const uint _N,
                   WaypointFile)
    : G(Graph(map_filename)), starts(Config()), goals(Config()), N(_N)
{
  std::ifstream file(wp_filename);
  if (!file) {
    info(0, 0, wp_filename, " is not found");
    return;
  }
  std::string line;
  while (starts.size() < N && getline(file, line)) {
    if (!line.empty() && line.back() == 0x0d) line.pop_back();  // CRLF
    if (std::regex_match(line, r_wp_comment)) continue;
    std::istringstream iss(line);
    std::vector<int> xy;
    int a;
    while (iss >> a) xy.push_back(a);
    if (!iss.eof() || xy.size() < 4 || xy.size() % 2 != 0) {
      info(0, 0, "invalid waypoint line: ", line);
      return;
    }
    Vertices seq;
    for (size_t k = 0; k < xy.size(); k += 2) {
      const auto x = xy[k], y = xy[k + 1];
      Vertex* v = nullptr;
      if (0 <= x && x < (int)G.width && 0 <= y && y < (int)G.height)
        v = G.U[G.width * y + x];
      if (v == nullptr) {
        info(0, 0, "invalid location (", x, ",", y, ") in: ", line);
        return;
      }
      seq.push_back(v);
    }
    starts.push_back(seq.front());
    seq.erase(seq.begin());
    goals.push_back(seq.back());
    waypoints.push_back(seq);
  }
}

Instance::Instance(const std::string& map_filename, std::mt19937* MT,
                   const uint _N, const uint m, const bool depot)
    : G(Graph(map_filename)), starts(Config()), goals(Config()), N(_N)
{
  // same procedure as the random instance for starts & final goals,
  // i.e., m=1 without depot reproduces the classical random instance
  const auto V_size = G.size();
  if (N > V_size || (!depot && m == 0)) return;

  auto s_indexes = std::vector<uint>(V_size);
  std::iota(s_indexes.begin(), s_indexes.end(), 0);
  std::shuffle(s_indexes.begin(), s_indexes.end(), *MT);
  for (uint i = 0; i < N; ++i) starts.push_back(G.V[s_indexes[i]]);

  if (depot) {
    goals = starts;
  } else {
    auto g_indexes = std::vector<uint>(V_size);
    std::iota(g_indexes.begin(), g_indexes.end(), 0);
    std::shuffle(g_indexes.begin(), g_indexes.end(), *MT);
    for (uint i = 0; i < N; ++i) goals.push_back(G.V[g_indexes[i]]);
  }

  // intermediate waypoints are not necessarily distinct
  std::uniform_int_distribution<uint> rand_v(0, V_size - 1);
  const auto num_intermediate = depot ? m : m - 1;
  for (uint i = 0; i < N; ++i) {
    waypoints.push_back(Vertices());
    for (uint k = 0; k < num_intermediate; ++k)
      waypoints[i].push_back(G.V[rand_v(*MT)]);
    waypoints[i].push_back(goals[i]);
  }
}

bool Instance::has_waypoints() const
{
  for (auto& seq : waypoints)
    if (seq.size() > 1) return true;
  return false;
}

void Instance::set_private_depots()
{
  reserved.assign(G.size(), -1);
  for (uint i = 0; i < N; ++i)
    if (starts[i] == goals[i]) reserved[starts[i]->id] = i;
}

void advance_K(const Instance& ins, const Config& C, std::vector<uint>& K)
{
  for (uint i = 0; i < ins.N; ++i) {
    while (K[i] + 1 < ins.m(i) && C[i] == ins.waypoints[i][K[i]]) ++K[i];
  }
}

std::vector<uint> initial_K(const Instance& ins)
{
  auto K = std::vector<uint>(ins.N, 0);
  advance_K(ins, ins.starts, K);
  return K;
}

bool Instance::is_valid(const int verbose) const
{
  if (N != starts.size() || N != goals.size()) {
    info(1, verbose, "invalid N, check instance");
    return false;
  }
  if (N != waypoints.size()) {
    info(1, verbose, "invalid waypoints, check instance");
    return false;
  }
  for (uint i = 0; i < N; ++i) {
    if (waypoints[i].empty() || goals[i] != waypoints[i].back()) {
      info(1, verbose, "invalid waypoints of agent ", i);
      return false;
    }
  }
  // waypoints on vertices reserved for other agents
  for (uint i = 0; i < N; ++i)
    for (auto w : waypoints[i])
      if (forbidden(i, w)) {
        info(1, verbose, "agent ", i, " has a waypoint reserved for another");
        return false;
      }
  // starts and final goals must be distinct
  for (auto C : {&starts, &goals}) {
    auto used = std::vector<bool>(G.size(), false);
    for (auto v : *C) {
      if (used[v->id]) {
        info(1, verbose, "duplicated start or final goal, check instance");
        return false;
      }
      used[v->id] = true;
    }
  }
  return true;
}

std::ostream& operator<<(std::ostream& os, const Solution& solution)
{
  auto N = solution.front().size();
  for (size_t i = 0; i < N; ++i) {
    os << std::setw(5) << i << ":";
    for (size_t k = 0; k < solution[i].size(); ++k) {
      if (k > 0) os << "->";
      os << std::setw(5) << solution[i][k]->index;
    }
    os << std::endl;
  }
  return os;
}
