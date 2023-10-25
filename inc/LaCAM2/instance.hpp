/*
 * instance definition
 */
#pragma once
#include <random>

#include "LaCAM2/graph.hpp"
#include "LaCAM2/utils.hpp"
#include "States.h"

namespace LaCAM2 {

struct AgentInfo {
public:
    int goal_location;
    float elapsed;
    float tie_breaker;
    int id;
    int stuck_order;

    AgentInfo():id(-1),goal_location(-1),elapsed(-1),tie_breaker(-1), stuck_order(0){};
};


struct Instance {
  const Graph & G;  // graph
  Config starts;  // initial configuration
  Config goals;   // goal configuration
  const uint N;   // number of agents
  vector<AgentInfo> & agent_infos;
  int planning_window=-1;
  std::vector<::Path> * precomputed_paths;

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
  Instance(
    const Graph & G,
    const std::vector<std::pair<uint,int> >& start_indexes,
    const std::vector<std::pair<uint,int> >& goal_indexes,
    std::vector<AgentInfo> & agent_infos,
    int planning_window=-1,
    std::vector<::Path> * precomputed_paths=nullptr
  );
  ~Instance() {}

  // simple feasibility check of instance
  bool is_valid(const int verbose = 0) const;
};

// solution: a sequence of configurations
using Solution = std::vector<Config>;
std::ostream& operator<<(std::ostream& os, const Solution& solution);

} // namespace LaCAM2