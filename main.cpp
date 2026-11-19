#include <argparse/argparse.hpp>
#include <lacam2.hpp>

int main(int argc, char* argv[])
{
  // arguments parser
  argparse::ArgumentParser program("lacam2", "0.1.0");
  program.add_argument("-m", "--map").help("map file").required();
  program.add_argument("-i", "--scen")
      .help("scenario file")
      .default_value(std::string(""));
  program.add_argument("-w", "--waypoints")
      .help("waypoint file, one agent per line: 'sx sy x1 y1 x2 y2 ...'")
      .default_value(std::string(""));
  program.add_argument("-W", "--random_waypoints")
      .help("random instance with this number of waypoints per agent")
      .default_value(std::string("0"));
  program.add_argument("--wp_depot")
      .help("random waypoints: return to the start location at the end")
      .default_value(false)
      .implicit_value(true);
  program.add_argument("--stn")
      .help("temporal constraints between waypoints (requires -w), see stn.hpp")
      .default_value(std::string(""));
  program.add_argument("--wp_private_depot")
      .help("the start of an agent returning there is reserved for it")
      .default_value(false)
      .implicit_value(true);
  program.add_argument("--wp_priority_reset")
      .help("reset PIBT priority at: each (waypoint) | final (waypoint)")
      .default_value(std::string("each"))
      .action([](const std::string& value) {
        if (value == "final" || value == "each") return value;
        throw std::runtime_error("--wp_priority_reset: final or each");
      });
  program.add_argument("-N", "--num").help("number of agents").required();
  program.add_argument("-s", "--seed")
      .help("seed")
      .default_value(std::string("0"));
  program.add_argument("-v", "--verbose")
      .help("verbose")
      .default_value(std::string("0"));
  program.add_argument("-t", "--time_limit_sec")
      .help("time limit sec")
      .default_value(std::string("3"));
  program.add_argument("-o", "--output")
      .help("output file")
      .default_value(std::string("./build/result.txt"));
  program.add_argument("-l", "--log_short")
      .default_value(false)
      .implicit_value(true);
  program.add_argument("-O", "--objective")
      .help("0: none, 1: makespan, 2: sum_of_loss")
      .default_value(std::string("0"))
      .action([](const std::string& value) {
        static const std::vector<std::string> C = {"0", "1", "2"};
        if (std::find(C.begin(), C.end(), value) != C.end()) return value;
        return std::string("0");
      });
  program.add_argument("-r", "--restart_rate")
      .help("restart rate")
      .default_value(std::string("0.001"));

  try {
    program.parse_known_args(argc, argv);
  } catch (const std::runtime_error& err) {
    std::cerr << err.what() << std::endl;
    std::cerr << program;
    std::exit(1);
  }

  // setup instance
  const auto verbose = std::stoi(program.get<std::string>("verbose"));
  const auto time_limit_sec =
      std::stoi(program.get<std::string>("time_limit_sec"));
  const auto scen_name = program.get<std::string>("scen");
  const auto seed = std::stoi(program.get<std::string>("seed"));
  auto MT = std::mt19937(seed);
  const auto map_name = program.get<std::string>("map");
  const auto output_name = program.get<std::string>("output");
  const auto log_short = program.get<bool>("log_short");
  const auto N = std::stoi(program.get<std::string>("num"));
  const auto wp_name = program.get<std::string>("waypoints");
  const auto wp_random = std::stoi(program.get<std::string>("random_waypoints"));
  const auto wp_depot = program.get<bool>("wp_depot");
  const auto wp_reset =
      program.get<std::string>("wp_priority_reset") == "final"
          ? WP_RESET_FINAL
          : WP_RESET_EACH;
  const auto stn_name = program.get<std::string>("stn");
  const auto private_depot = program.get<bool>("wp_private_depot");
  auto ins =
      wp_name.size() > 0 ? Instance(map_name, wp_name, N, WaypointFile())
      : scen_name.size() > 0 ? Instance(scen_name, map_name, N)
      : (wp_random > 0 || wp_depot)
          ? Instance(map_name, &MT, N, std::max(wp_random, 1), wp_depot)
          : Instance(map_name, &MT, N);
  const auto objective =
      static_cast<Objective>(std::stoi(program.get<std::string>("objective")));
  const auto restart_rate = std::stof(program.get<std::string>("restart_rate"));
  if (private_depot) ins.set_private_depots();
  if (!ins.is_valid(1)) return 1;
  if (!stn_name.empty()) {
    if (wp_name.empty()) {
      std::cerr << "--stn requires a waypoint file (-w)" << std::endl;
      return 1;
    }
    auto stn = std::make_shared<TemporalNetwork>(stn_name, ins);
    if (!stn->valid) {
      std::cerr << "invalid STN " << stn_name << ": " << stn->error
                << std::endl;
      return 1;
    }
    ins.stn = stn;
  }

  // solve
  auto additional_info = std::string("");
  const auto deadline = Deadline(time_limit_sec * 1000);
  const auto solution = solve(ins, additional_info, verbose - 1, &deadline, &MT,
                              objective, restart_rate, wp_reset);
  const auto comp_time_ms = deadline.elapsed_ms();

  // failure
  if (solution.empty()) info(1, verbose, "failed to solve");

  // check feasibility
  if (!is_feasible_solution(ins, solution, verbose)) {
    info(0, verbose, "invalid solution");
  }

  // post processing
  print_stats(verbose, ins, solution, comp_time_ms);
  make_log(ins, solution, output_name, comp_time_ms, map_name, seed,
           additional_info, log_short);
  return 0;
}
