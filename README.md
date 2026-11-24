lacam2
---
[![MIT License](http://img.shields.io/badge/license-MIT-blue.svg?style=flat)](LICENSE)
[![CI](https://github.com/Kei18/lacam2/actions/workflows/ci.yml/badge.svg)](https://github.com/Kei18/lacam2/actions/workflows/ci.yml)

Do you want the power?
LaCAM* could be the answer.

This is a code repository of the paper ["Improving LaCAM for Scalable Eventually Optimal Multi-Agent Pathfinding"](https://kei18.github.io/lacam2/) (IJCAI-23), extended from from [the LaCAM repo](https://kei18.github.io/lacam/) presented at AAAI-23.

__A refactored, clean version is available: [lacam0](https://github.com/Kei18/lacam0). I recommend using it instead of this repo.__

## Building

All you need is [CMake](https://cmake.org/) (≥v3.16). The code is written in C++(17).

First, clone this repo with submodules.

```sh
git clone --recursive https://github.com/Kei18/lacam2.git && cd lacam2
```

Then, build the project.

```sh
cmake -B build && make -C build
```

You can also use the [docker](https://www.docker.com/) environment (based on Ubuntu18.04) instead of the native one.
An example setup is available in `assets/`.

## Usage

no optimization (random starts/goals):

```sh
> build/main -v 1 -m assets/random-32-32-20.map -N 400
solved: 31ms    makespan: 112 (lb=58, ub=1.94)  sum_of_costs: 31373 (lb=9217, ub=3.41)  sum_of_loss: 26001 (lb=9217, ub=2.83)

# with the MAPF visualizer mentioned below
> mapf-visualizer map/random-32-32-20.map build/result.txt
```

![](assets/demo-random-32-32-20_400agents.gif)

makespan optimization:

```sh
> build/main -m assets/loop.map -i assets/loop.scen -N 3 -v 1 --objective 1
solved: 8ms     makespan: 10 (lb=2, ub=5)       sum_of_costs: 21 (lb=5, ub=4.2) sum_of_loss: 21 (lb=5, ub=4.2)
```

sum-of-loss optimization:

```sh
> build/main -m assets/loop.map -i assets/loop.scen -N 3 -v 2 --objective 2
solved: 1ms     makespan: 11 (lb=2, ub=5.5)     sum_of_costs: 15 (lb=5, ub=3)   sum_of_loss: 15 (lb=5, ub=3)
```

You can find details of all parameters with:
```sh
build/main --help
```

## Waypoint sequences

Each agent can visit an ordered sequence of waypoints; the last one is its final goal.
The high-level search state becomes `(Q, K)`: the configuration `Q` plus, for each agent, the index `K[i]` of its next waypoint.

From a file, one agent per line, `sx sy x1 y1 x2 y2 ...` (see `assets/waypoints-depot.wp`):

```sh
> build/main -m assets/empty-8-8.map -w assets/waypoints-depot.wp -N 1 -v 1
solved: 0ms     makespan: 6 (lb=6, ub=1)        sum_of_costs: 6 (lb=6, ub=1)    sum_of_loss: 6 (lb=6, ub=1)
```

Random instances with `M` waypoints per agent (`--wp_depot` adds the return to the start location):

```sh
> build/main -m assets/random-32-32-20.map -N 50 -W 4 --wp_depot -v 1
```

Options:
- `--wp_priority_reset each|final` (default `each`): the PIBT priority is reset at every reached waypoint, or only when the whole sequence is completed.
- `--objective 1|2`: makespan or sum-of-loss (as in LaCAM* but for the whole sequence).

The output file has an additional `waypoints=` line, `(x,y);(x,y)|...` per agent.


## Temporal constraints (STN)

With `--stn FILE` (together with `-w`), waypoints are linked by a Simple Temporal Network, with precedence constraints and absolute max delays.
An event `(i, k)` is the instant agent `i` registers its `k`-th intermediate waypoint (the final one has no event).
It is triggered at the first instant the agent is on the waypoint with its window open and all the events that must precede it fired; an agent arriving early waits, on the cell or elsewhere.

```
# lb <= t_b - t_a <= ub, timepoint 0 is the origin (time 0), '-' unbounded
timepoints 3
0 0        # 1: agent 0, waypoint 0
-1 0       # 2: free timepoint (e.g., an operation), projected away
1 0        # 3: agent 1, waypoint 0
constraints 3
1 2 2 -
2 3 1 -    # agent 1 registers its waypoint at least 3 steps after agent 0
0 3 - 40   # ... and by time 40
```

- Supported: lower bounds between events, absolute windows. 
- The search state is `(Q, K, W)`, where `W` holds the windows of the pending events relative to the current time step.
- The output file has an additional `events=` line, the time each waypoint is registered, `t;t;...|...` per agent.
- `soc_lb_stn` and `makespan_lb_stn` in the output file are lower bounds that include the waits imposed by the STN (earliest firing time of every event, precedences between agents included), unlike `soc_lb` and `makespan_lb`; without STN they are equal.


## Visualizer

This repository is compatible with [@Kei18/mapf-visualizer](https://github.com/kei18/mapf-visualizer).

## Licence

This software is released under the MIT License, see [LICENSE.txt](LICENCE.txt).
