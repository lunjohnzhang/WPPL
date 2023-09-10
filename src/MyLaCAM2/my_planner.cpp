#include "MyLaCAM2/my_planner.hpp"

namespace MyLaCAM2 {

uint MyHNode::HNODE_CNT = 0;

// for high-level
MyHNode::MyHNode(const MyConfig& _C, const std::shared_ptr<HeuristicTable> & HT, const MyInstance * ins, 
    MyHNode* _parent, const uint _g, const uint _h, bool use_dist_in_priority)
    : C(_C),
      parent(_parent),
      neighbor(),
      g(_g),
      h(_h),
      f(g + h),
      priorities(C.size()),
      order(C.size(), 0)
{
  ++HNODE_CNT;

  const auto N = C.size();

  // update neighbor
  if (parent != nullptr) parent->neighbor.insert(this);

  // set order
  std::iota(order.begin(), order.end(), 0);
  std::sort(order.begin(), order.end(), [&](int i, int j) { 
        const MyAgentInfo & a=ins->agent_infos[i];
        const MyAgentInfo & b=ins->agent_infos[j];

        if (use_dist_in_priority) {
          int h1=HT->get(C[i]->v->index,a.goal_location);
          int h2=HT->get(C[j]->v->index,b.goal_location);

          if (h1!=h2) return h1<h2;
        }

        if (a.elapsed!=b.elapsed) return a.elapsed>b.elapsed;

        return a.tie_breaker>b.tie_breaker;
  });

}

MyHNode::~MyHNode()
{
}

MyPlanner::MyPlanner(const MyInstance* _ins, const std::shared_ptr<HeuristicTable> & HT, const Deadline* _deadline, std::mt19937* _MT, 
                 const int _verbose,
                 const Objective _objective, const float _restart_rate,
                 bool use_orient_in_heuristic, bool use_dist_in_priority)
    : ins(_ins),
      deadline(_deadline),
      MT(_MT),
      verbose(_verbose),
      objective(_objective),
      RESTART_RATE(_restart_rate),
      N(ins->N),
      V_size(ins->G.size()),
      HT(HT),
      loop_cnt(0),
      C_next(N),
      tie_breakers(V_size, 0),
      A(N, nullptr),
      occupied_now(V_size, nullptr),
      occupied_next(V_size, nullptr),
      use_orient_in_heuristic(use_orient_in_heuristic),
      use_dist_in_priority(use_dist_in_priority)
{
}

MyPlanner::~MyPlanner() {}

MySolution MyPlanner::solve(std::string& additional_info)
{
  solver_info(1, "start search");

  // setup agents
  for (auto i = 0; i < N; ++i) A[i] = new MyAgent(i);

  // insert initial node, 'H': high-level node
  auto H_init = new MyHNode(ins->starts, HT, ins, nullptr, 0, get_h_value(ins->starts), use_dist_in_priority);

  // cerr<<"starts:";
  // for (auto start:ins->starts){
  //   cerr<<start->v->index<<",";
  // }
  // cerr<<endl;

  std::vector<MyConfig> solution;
  auto C_new = MyConfig(N, nullptr);  // for new configuration
  MyHNode* H_goal = nullptr;          // to store goal node

  // create successors at the high-level search
  const auto res = get_new_config(H_init);
  if (!res) {
    // TODO
  }

  // create new configuration
  for (auto a : A) C_new[a->id] = a->v_next;

  H_goal = new MyHNode(C_new, HT, ins, H_init, 0, get_h_value(C_new), use_dist_in_priority); 


  // backtrack
  if (H_goal != nullptr) {
    auto H = H_goal;
    while (H != nullptr) {
      solution.push_back(H->C);
      H = H->parent;
    }
    std::reverse(solution.begin(), solution.end());
  }

  solver_info(1, "solved, objective: ", objective);

  // print result
  // if (H_goal != nullptr && OPEN.empty()) {
  //   solver_info(1, "solved optimally, objective: ", objective);
  // } else if (H_goal != nullptr) {
  //   solver_info(1, "solved sub-optimally, objective: ", objective);
  // } else if (OPEN.empty()) {
  //   solver_info(1, "no solution");
  // } else {
  //   solver_info(1, "timeout");
  // }

  // memory management
  for (auto a : A) delete a;
  delete H_goal;
  delete H_init;

  return solution;
}

uint MyPlanner::get_edge_cost(const MyConfig& C1, const MyConfig& C2)
{
  if (objective == OBJ_SUM_OF_LOSS) {
    uint cost = 0;
    for (uint i = 0; i < N; ++i) {
      if (C1[i]->v->index != ins->agent_infos[i].goal_location || C2[i]->v->index != ins->agent_infos[i].goal_location) {
        cost += 1;
      }
    }
    return cost;
  }

  // default: makespan
  return 1;
}

uint MyPlanner::get_edge_cost(MyHNode* H_from, MyHNode* H_to)
{
  return get_edge_cost(H_from->C, H_to->C);
}

uint MyPlanner::get_h_value(const MyConfig& C)
{
  uint cost = 0;
  if (objective == OBJ_MAKESPAN) {
    for (auto i = 0; i < N; ++i) cost = std::max(cost, HT->get(C[i]->v->index, ins->agent_infos[i].goal_location));
  } else if (objective == OBJ_SUM_OF_LOSS) {
    for (auto i = 0; i < N; ++i) cost += HT->get(C[i]->v->index, ins->agent_infos[i].goal_location);
  }
  return cost;
}

bool MyPlanner::get_new_config(MyHNode* H)
{
  // setup cache
  for (auto a : A) {
    // clear previous cache
    if (a->v_now != nullptr && occupied_now[a->v_now->v->id] == a) {
      occupied_now[a->v_now->v->id] = nullptr;
    }
    if (a->v_next != nullptr) {
      occupied_next[a->v_next->v->id] = nullptr;
      a->v_next = nullptr;
    }

    // set occupied now
    a->v_now = H->C[a->id];
    occupied_now[a->v_now->v->id] = a;
  }

  // perform PIBT
  for (auto k : H->order) {
    auto a = A[k];
    if (a->v_next == nullptr && !funcPIBT(a)) return false;  // planning failure
  }
  return true;
}

int MyPlanner::get_neighbor_orientation(const Graph & G, int loc1, int loc2) {

    // 0:east, 1:south, 2:west, 3:north

    if (loc1+1==loc2) {
        return 0;
    }

    if (loc1+G.width==loc2) {
        return 1;
    }

    if (loc1-1==loc2) {
        return 2;
    }

    if (loc1-G.width==loc2) {
        return 3;
    }

    // TODO we cannot return 5!
    if (loc1==loc2) {
      return 5;
    }

    std::cerr<<"loc1 and loc2 are not neighbors: "<<loc1<<", "<<loc2<<endl;
    exit(-1);
    return -1;
}

bool MyPlanner::funcPIBT(MyAgent* ai)
{
  const auto i = ai->id;
  const auto K = ai->v_now->v->neighbor.size();

  // get candidates for next locations
  for (auto k = 0; k < K; ++k) {
    auto u = ai->v_now->v->neighbor[k];  
    // TODO(hj): we need to manage the memory of state later
    auto state = new MyState(u);
    C_next[i][k] = state;
    if (MT != nullptr)
      tie_breakers[state->v->id] = get_random_float(MT);  // set tie-breaker
  }
  C_next[i][K] = ai->v_now;
  tie_breakers[ai->v_now->v->id] = get_random_float(MT);  // set tie-breaker

  // sort
  std::sort(C_next[i].begin(), C_next[i].begin() + K + 1,
            [&](MyState* const s1, MyState* const s2) {

    int o1=get_neighbor_orientation(ins->G,ai->v_now->v->index,s1->v->index);
    int o2=get_neighbor_orientation(ins->G,ai->v_now->v->index,s2->v->index);

    double d1,d2;
    if (use_orient_in_heuristic){
      d1=HT->get(s1->v->index,o1,(ins->agent_infos[i]).goal_location);
      d2=HT->get(s2->v->index,o2,(ins->agent_infos[i]).goal_location);      
    } else {
      d1=HT->get(s1->v->index,(ins->agent_infos[i]).goal_location);
      d2=HT->get(s2->v->index,(ins->agent_infos[i]).goal_location);
    }

    if (d1!=d2) return d1<d2;

    return o1<o2;

  });

  // cerr<<ai->id<<" "<<ai->v_now->v->index<<endl;
  // for (auto k=0;k<K+1;++k) {
  //   cerr<<C_next[i][k]->v->index<<",";
  // }
  // cerr<<endl;

  // main operation
  for (auto k = 0; k < K + 1; ++k) {
    auto s = C_next[i][k];

    // avoid vertex conflicts
    if (occupied_next[s->v->id] != nullptr) continue;

    auto& ak = occupied_now[s->v->id];

    // avoid swap conflicts
    if (ak != nullptr && ak->v_next != nullptr && ak->v_next->v->id == ai->v_now->v->id) continue;

    // reserve next location
    occupied_next[s->v->id] = ai;
    ai->v_next = s;

    // priority inheritance
    if (ak != nullptr && ak != ai && ak->v_next == nullptr && !funcPIBT(ak))
      continue;

    // success to plan next one step
    return true;
  }

  // failed to secure node
  occupied_next[ai->v_now->v->id] = ai;
  ai->v_next = ai->v_now;
  return false;
}

}  // namespace MyLaCAM2