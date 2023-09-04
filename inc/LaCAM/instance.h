/*
 * instance definition
 */
#pragma once
#include <random>

#include "LaCAM/graph.h"
#include "LaCAM/utils.h"
#include "RHCR/interface/CompetitionGraph.h"
#include "common.h"

namespace LaCAM {

struct Instance {
  const Graph & G;  // graph
  Config starts;  // initial configuration
  Config goals;   // goal configuration
  const uint N;   // number of agents
  vector<int> priorities;

  // // for testing
  // Instance(const std::string& map_filename,
  //          const std::vector<int>& start_indexes,
  //          const std::vector<int>& goal_indexes);
  // // for MAPF benchmark
  // Instance(const std::string& scen_filename, const std::string& map_filename,
  //          const int _N = 1);
  // // random instance generation
  // Instance(const std::string& map_filename, std::mt19937* MT, const int _N = 1);
  // for Competition
  Instance(const Graph & G,
          const std::vector<int>& start_indexes,
          const std::vector<int>& goal_indexes,
          const std::vector<int>& priorities);
  ~Instance() {}

  // simple feasibility check of instance
  bool is_valid(const int verbose = 0) const;
};

// solution: a sequence of configurations
using Solution = std::vector<Config>;

}
