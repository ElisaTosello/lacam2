/*
 * utility functions
 */
#pragma once

#include <array>
#include <chrono>
#include <climits>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <numeric>
#include <queue>
#include <random>
#include <regex>
#include <set>
#include <sstream>
#include <stack>
#include <string>
#include <tuple>
#include <unordered_map>
#include <vector>

using Time = std::chrono::steady_clock;

void info(const int level, const int verbose);

template <typename Head, typename... Tail>
void info(const int level, const int verbose, Head&& head, Tail&&... tail)
{
  if (verbose < level) return;
  std::cout << head;
  info(level, verbose, std::forward<Tail>(tail)...);
}

// time manager
// is_expired: the time limit is reached or, with mem_limit_mb > 0, the
// virtual memory of the process exceeds it (sampled every MEM_CHECK_PERIOD
// calls); then memory_exceeded tells which
struct Deadline {
  static constexpr unsigned int MEM_CHECK_PERIOD = 1024;
  const Time::time_point t_s;
  const double time_limit_ms;
  const double mem_limit_mb;  // 0: no limit
  mutable unsigned int calls;
  mutable bool memory_exceeded;

  Deadline(double _time_limit_ms = 0, double _mem_limit_mb = 0);
  double elapsed_ms() const;
  double elapsed_ns() const;
};

// virtual memory of the process [MB], 0 if unknown (/proc/self/statm)
double get_memory_mb();

double elapsed_ms(const Deadline* deadline);
double elapsed_ns(const Deadline* deadline);
bool is_expired(const Deadline* deadline);

float get_random_float(std::mt19937* MT, float from = 0, float to = 1);
int get_random_int(std::mt19937* MT, int from = 0, int to = 1);
