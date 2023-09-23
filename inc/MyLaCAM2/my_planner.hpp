/*
 * lacam-star
 */

#pragma once

#include "MyLaCAM2/graph.hpp"
#include "MyLaCAM2/instance.hpp"
#include "MyLaCAM2/utils.hpp"
#include "MyLaCAM2/my_state.hpp"
#include "util/HeuristicTable.h"
#include "MyLaCAM2/planner.hpp"
#include "MyLaCAM2/my_instance.hpp"
#include <memory>

namespace MyLaCAM2 {

// PIBT agent
struct MyAgent {
  const uint id;
  MyState* v_now;   // current state
  MyState* v_next;  // next state
  MyAgent(uint _id) : id(_id), v_now(nullptr), v_next(nullptr) {}
};
using MyAgents = std::vector<MyAgent*>;

// high-level node
struct MyHNode {
  static uint HNODE_CNT;  // count #(high-level node)
  const MyConfig C;

  // tree
  MyHNode* parent;
  std::set<MyHNode*> neighbor;

  // costs
  uint g;        // g-value (might be updated)
  const uint h;  // h-value
  uint f;        // g + h (might be updated)

  // for low-level search
  std::vector<float> priorities;
  std::vector<uint> order;

  MyHNode(const MyConfig& _C, const std::shared_ptr<HeuristicTable> & HT, const MyInstance * ins, 
          MyHNode* _parent, const uint _g, const uint _h, bool use_dist_in_priority);
  ~MyHNode();
};

using MyHNodes = std::vector<MyHNode*>;

struct MyPlanner {
  const MyInstance* ins;
  const Deadline* deadline;
  std::mt19937* MT;
  const int verbose;
  bool use_orient_in_heuristic=true;  // use orientation in heuristic or not
  bool use_dist_in_priority=true; // use distance in priority or not

  // hyper parameters
  const Objective objective;
  const float RESTART_RATE;  // random restart

  // solver utils
  const uint N;       // number of agents
  const uint V_size;  // number o vertices
  std::shared_ptr<HeuristicTable> HT;
  uint loop_cnt;      // auxiliary

  // used in PIBT
  std::vector<std::array<MyState*, 5> > C_next;  // next locations, used in PIBT
  std::vector<float> tie_breakers;              // random values, used in PIBT
  MyAgents A;
  MyAgents occupied_now;                          // for quick collision checking
  MyAgents occupied_next;                         // for quick collision checking

  MyPlanner(const MyInstance* _ins, const std::shared_ptr<HeuristicTable> & HT, const Deadline* _deadline, std::mt19937* _MT,
          const int _verbose = 0,
          // other parameters
          const Objective _objective = OBJ_NONE,
          const float _restart_rate = 0.001,
          bool use_orient_in_heuristic=true,
          bool use_dist_in_priority=true);
  ~MyPlanner();
  MySolution solve(std::string& additional_info);
  uint get_edge_cost(const MyConfig& C1, const MyConfig& C2);
  uint get_edge_cost(MyHNode* H_from, MyHNode* H_to);
  uint get_h_value(const MyConfig& C);
  bool get_new_config(MyHNode* H);
  bool funcPIBT(MyAgent* ai);
  int get_neighbor_orientation(const Graph & G, int loc1, int loc2);

  // utilities
  template <typename... Body>
  void solver_info(const int level, Body&&... body)
  {
    if (verbose < level) return;
    std::cout << "elapsed:" << std::setw(6) << elapsed_ms(deadline) << "ms"
              << "  loop_cnt:" << std::setw(8) << loop_cnt
              << "  node_cnt:" << std::setw(8) << MyHNode::HNODE_CNT << "\t";
    info(level, verbose, (body)...);
  }
};

}  // namespace MyLaCAM2