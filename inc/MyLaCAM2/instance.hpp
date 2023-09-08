/*
 * instance definition
 */
#pragma once
#include <random>

#include "MyLaCAM2/graph.hpp"
#include "MyLaCAM2/utils.hpp"

namespace MyLaCAM2 {

struct AgentInfo {
public:
    int goal_location;
    float elapsed;
    float tie_breaker;

    AgentInfo():goal_location(-1),elapsed(-1),tie_breaker(-1){};
};


struct Instance {
  const Graph & G;  // graph
  Config starts;  // initial configuration
  Config goals;   // goal configuration
  const uint N;   // number of agents
  const vector<AgentInfo> & agent_infos;

  // // for testing
  // Instance(const std::string& map_filename,
  //          const std::vector<uint>& start_indexes,
  //          const std::vector<uint>& goal_indexes);
  // // for MAPF benchmark
  // Instance(const std::string& scen_filename, const std::string& map_filename,
  //          const uint _N = 1);
  // // random instance generation
  // Instance(const std::string& map_filename, std::mt19937* MT,
  //          const uint _N = 1);
  Instance(const Graph & G,
          const std::vector<uint>& start_indexes,
          const std::vector<uint>& goal_indexes,
          const std::vector<AgentInfo> & agent_infos);
  ~Instance() {}

  // simple feasibility check of instance
  bool is_valid(const int verbose = 0) const;
};

// solution: a sequence of configurations
using Solution = std::vector<Config>;
std::ostream& operator<<(std::ostream& os, const Solution& solution);

} // namespace MyLaCAM2