#include "../include/utils.hpp"

#include <unistd.h>

void info(const int level, const int verbose) { std::cout << std::endl; }

Deadline::Deadline(double _time_limit_ms, double _mem_limit_mb)
    : t_s(Time::now()),
      time_limit_ms(_time_limit_ms),
      mem_limit_mb(_mem_limit_mb),
      calls(0),
      memory_exceeded(false)
{
}

double get_memory_mb()
{
  std::ifstream file("/proc/self/statm");
  long long pages = 0;
  if (!(file >> pages)) return 0;
  return (double)pages * sysconf(_SC_PAGESIZE) / (1 << 20);
}

double Deadline::elapsed_ms() const
{
  return std::chrono::duration_cast<std::chrono::milliseconds>(Time::now() -
                                                               t_s)
      .count();
}

double Deadline::elapsed_ns() const
{
  return std::chrono::duration_cast<std::chrono::nanoseconds>(Time::now() - t_s)
      .count();
}

double elapsed_ms(const Deadline* deadline)
{
  if (deadline == nullptr) return 0;
  return deadline->elapsed_ms();
}

double elapsed_ns(const Deadline* deadline)
{
  if (deadline == nullptr) return 0;
  return deadline->elapsed_ns();
}

bool is_expired(const Deadline* deadline)
{
  if (deadline == nullptr) return false;
  if (deadline->memory_exceeded) return true;
  if (deadline->mem_limit_mb > 0 &&
      ++deadline->calls % Deadline::MEM_CHECK_PERIOD == 0 &&
      get_memory_mb() > deadline->mem_limit_mb) {
    deadline->memory_exceeded = true;
    return true;
  }
  return deadline->elapsed_ms() > deadline->time_limit_ms;
}

float get_random_float(std::mt19937* MT, float from, float to)
{
  std::uniform_real_distribution<float> r(from, to);
  return r(*MT);
}

int get_random_int(std::mt19937* MT, int from, int to)
{
  std::uniform_int_distribution<int> r(from, to);
  return r(*MT);
}
