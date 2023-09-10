# pragma once
# include "MyLaCAM2/utils.hpp"
# include "MyLaCAM2/graph.hpp"
# include "common.h"

namespace MyLaCAM2 {

struct MyState {
  Vertex * v;
  int orient; // current orientation
  int goal_id; // goal_id: 1 means having arrived the first goal
  MyState(Vertex * v=nullptr, int orient=-1, int goal_id=-1): 
    v(v), orient(orient), goal_id(goal_id) {
  }
  MyState(const MyState & s): v(s.v), orient(s.orient), goal_id(s.goal_id) {
    std::cerr<<"copied"<<endl;
  }
};

using MyConfig = std::vector<MyState*>;
using MySolution = std::vector<MyConfig>;

// bool is_same_config(
//     const MyConfig& C1,
//     const MyConfig& C2) {
  
//   const auto N = C1.size();
  
//   for (size_t i = 0; i < N; ++i) {
//     if (C1[i]->v->index != C2[i]->v->index || C1[i]->orient!=C2[i]->orient || C1[i]->goal_id!=C2[i]->goal_id) return false;
//   }
//   return true;

// };  // check equivalence of two configurations

// hash function of configuration
// c.f.
// https://stackoverflow.com/questions/10405030/c-unordered-map-fail-when-used-with-a-vector-as-key
struct MyConfigHasher {
    uint operator()(const MyConfig& C) const {
        uint hash = C.size();
        for (auto& s : C) hash ^= 1000000*s->goal_id + 5*s->v->index + s->orient + 0x9e3779b9 + (hash << 6) + (hash >> 2);
        return hash;
    }
};

// std::ostream& operator<<(std::ostream& os, const MyState & state)
// {
//     os<<state.v->index<<","<<state.orient<<","<<state.goal_id;
//     return os;
// }

// std::ostream& operator<<(std::ostream& os, const MyConfig& config)
// {
//   os << "<";
//   const auto N = config.size();
//   for (size_t i = 0; i < N; ++i) {
//     if (i > 0) os << " ";
//     os << "[" << i << "]" << config[i];
//   }
//   os << ">";
//   return os;
// }

// std::ostream& operator<<(std::ostream& os, const MySolution& solution)
// {
//   auto N = solution.front().size();
//   for (size_t i = 0; i < N; ++i) {
//     os << i << ":";
//     for (size_t k = 0; k < solution[i].size(); ++k) {
//       if (k > 0) os << "->";
//       os << solution[i][k];
//     }
//     os << std::endl;
//   }
//   return os;
// }
    
}  // namespace MyLaCAM2
