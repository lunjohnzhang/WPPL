/*
 * instance definition
 */
#pragma once
#include <random>

#include "MyLaCAM2/graph.hpp"
#include "MyLaCAM2/utils.hpp"
#include "MyLaCAM2/my_state.hpp"

namespace MyLaCAM2 {

struct MyAgentInfo {
public:
    MyState start_state;
    int goal_location;
    float elapsed;
    float tie_breaker;

    MyAgentInfo(const MyState & start_state=MyState(), int goal_location=-1, float elapsed=-1, float tie_breaker=-1):
      start_state(start_state),goal_location(goal_location),elapsed(elapsed),tie_breaker(tie_breaker){
    };

    MyAgentInfo(const MyAgentInfo & agent_info):
      start_state(agent_info.start_state),goal_location(agent_info.goal_location),elapsed(agent_info.elapsed),tie_breaker(agent_info.tie_breaker){
        // std::cerr<<"super copied "<<agent_info.start_state.v->index<<endl;
    };
};


struct MyInstance {
  const Graph & G;  // graph
  const uint N;   // number of agents
  vector<MyAgentInfo> & agent_infos;
  MyConfig starts;

  MyInstance(const Graph & G,std::vector<MyAgentInfo>& agent_infos):
      G(G),
      N(agent_infos.size()),
      agent_infos(agent_infos) {
        for (auto i=0;i<agent_infos.size();++i) {
            starts.push_back(&(agent_infos[i].start_state));
            // std::cerr<<"here "<<starts[i]->v->index<<endl;
        }
  }
  ~MyInstance() {}
};

} // namespace MyLaCAM2