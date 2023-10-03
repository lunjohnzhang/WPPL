#include "LaCAM2/planner.hpp"

namespace LaCAM2 {

LNode::LNode(LNode* parent, uint i, Vertex* v)
    : who(), where(), depth(parent == nullptr ? 0 : parent->depth + 1)
{
  if (parent != nullptr) {
    who = parent->who;
    who.push_back(i);
    where = parent->where;
    where.push_back(v);
  }
}

uint HNode::HNODE_CNT = 0;

// for high-level
HNode::HNode(const Config& _C, const std::shared_ptr<HeuristicTable> & HT, const Instance * ins, HNode* _parent, const int _g,
             const int _h, const uint _d)
    : C(_C),
      parent(_parent),
      neighbor(),
      g(_g),
      h(_h),
      d(_d),
      f(g + h),
      priorities(C.size()),
      order(C.size(), 0),
      search_tree(std::queue<LNode*>())
{
  ++HNODE_CNT;

  const auto N = C.size();

  // update neighbor
  if (parent != nullptr) parent->neighbor.insert(this);

  // set order
  std::iota(order.begin(), order.end(), 0);
  std::sort(order.begin(), order.end(), [&](int i, int j) { 
        const AgentInfo & a=ins->agent_infos[i];
        const AgentInfo & b=ins->agent_infos[j];

        if (ins->precomputed_paths!=nullptr){
          bool precomputed_a = (*(ins->precomputed_paths))[i].size()>(d+1);
          bool precomputed_b = (*(ins->precomputed_paths))[j].size()>(d+1);
          if (precomputed_a != precomputed_b) return (int)precomputed_a>(int)precomputed_b;
        }

        int h1=HT->get(C[i]->index,ins->goals[i]->index);
        int h2=HT->get(C[j]->index,ins->goals[j]->index);

        if (h1!=h2) return h1<h2;

        if (a.elapsed!=b.elapsed) return a.elapsed>b.elapsed;

        return a.tie_breaker>b.tie_breaker;
  });

  search_tree.push(new LNode());

  // if (ins->precomputed_paths!=nullptr) {
  //   // low-level tree
  //   int llt_depth = 0;
  //   while (llt_depth<N) {
  //     int aid = order[llt_depth];
  //     auto & path = (*(ins->precomputed_paths))[aid];
  //     bool precomputed = path.size()>(d+1);
  //     if (precomputed) {
  //       auto L = search_tree.front();
  //       search_tree.pop();
  //       auto v = ins->G.U[path[d+1].location];
  //       search_tree.push(new LNode(L, aid, v));
  //       delete L; // free it, it won't be used anymore.
  //     } else {
  //       break;
  //     }
  //     llt_depth+=1;
  //   }
  // }
}

HNode::~HNode()
{
  while (!search_tree.empty()) {
    delete search_tree.front();
    search_tree.pop();
  }
}

Planner::Planner(const Instance* _ins, const std::shared_ptr<HeuristicTable> & HT, const Deadline* _deadline,
                 std::mt19937* _MT, const int _verbose,
                 const Objective _objective, const float _restart_rate, bool use_swap, bool use_orient_in_heuristic)
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
      use_swap(use_swap),
      use_orient_in_heuristic(use_orient_in_heuristic)
{
}

Planner::~Planner() {}

Solution Planner::solve(std::string& additional_info)
{
  solver_info(1, "start search");

  // setup agents
  for (auto i = 0; i < N; ++i) A[i] = new Agent(i);

  // setup search
  auto OPEN = std::stack<HNode*>();
  auto EXPLORED = std::unordered_map<Config, HNode*, ConfigHasher>();
  // insert initial node, 'H': high-level node
  auto H_init = new HNode(ins->starts, HT, ins, nullptr, 0, get_h_value(ins->starts), 0);
  OPEN.push(H_init);
  EXPLORED[H_init->C] = H_init;

  std::vector<Config> solution;
  auto C_new = Config(N, nullptr);  // for new configuration
  HNode* H_goal = nullptr;          // to store goal node


  int d_max=INT_MAX/2;
  if (ins->planning_window>0) {
    d_max=ins->planning_window;
  }

  // DFS
  while (!OPEN.empty() && !is_expired(deadline)) {
    loop_cnt += 1;

    // do not pop here!
    auto H = OPEN.top();  // high-level node

    // only plan for one step now
    // check goal condition: only require someone to arrive in lifelong case
    // bool someone_arrived = false;
    // for (int i=0;i<N;++i) {
    //   if (H->C[i]->id == ins->goals[i]->id || loop_cnt>1){
    //     someone_arrived = true;
    //     break;
    //   }
    // }

    // if (someone_arrived || loop_cnt>1) {
    //   H_goal=H;
    //   break;
    // }

    // if we find a depth d_max valid solution, then we just return it
    if (H->d>=d_max) {
      H_goal=H;
      break;
    }

    // low-level search end
    if (H->search_tree.empty()) {
      OPEN.pop();
      continue;
    }

    // check lower bounds
    if (H_goal != nullptr && H->f >= H_goal->f) {
      OPEN.pop();
      continue;
    }

    // check goal condition
    if (H_goal == nullptr && is_same_config(H->C, ins->goals)) {
      H_goal = H;
      solver_info(1, "found solution, cost: ", H->g);
      if (objective == OBJ_NONE) break;
      continue;
    }

    // create successors at the low-level search
    auto L = H->search_tree.front();
    H->search_tree.pop();
    expand_lowlevel_tree(H, L);

    // create successors at the high-level search
    const int num_trials=1;
    bool found = false;
    auto _C_new = Config(N, nullptr);  // for new configuration
    for (MC_idx=0;MC_idx<num_trials;++MC_idx){
      const auto res = get_new_config(H, L);

      int _h_val=-1;
      int h_val=-1;

      if (res){
        for (auto a : A) _C_new[a->id] = a->v_next;
        _h_val=get_h_value(_C_new);
        // cerr<<MC_idx<<" "<<_h_val<<endl;
      }

      if (!found) {
        C_new=_C_new;
      } else {
        h_val=get_h_value(C_new);
        // if (_h_val<h_val) {
        //   C_new=_C_new;
        //   // cerr<<MC_idx<<" best: "<<_h_val<<endl;
        // } else if (_h_val==h_val) {
          for (auto aid: H->order) {
            int h=HT->get(C_new[aid]->index,ins->goals[aid]->index);
            int _h=HT->get(_C_new[aid]->index,ins->goals[aid]->index);
            if (_h<h) {
              C_new=_C_new;
              break;
            }
          }
        // }
      }

      if (res) found=true;
    }

    delete L;  // free
    if (!found) {
      continue;
    }

    // create new configuration
    // for (auto a : A) C_new[a->id] = a->v_next;

    // check explored list
    const auto iter = EXPLORED.find(C_new);
    if (iter != EXPLORED.end()) {
      // case found
      rewrite(H, iter->second, H_goal, OPEN);
      // re-insert or random-restart
      auto H_insert = (MT != nullptr && get_random_float(MT) >= RESTART_RATE)
                          ? iter->second
                          : H_init;
      if (H_goal == nullptr || H_insert->f < H_goal->f) OPEN.push(H_insert);
    } else {
      // insert new search node
      const auto H_new = new HNode(
          C_new, HT, ins, H, H->g + get_edge_cost(H->C, C_new), get_h_value(C_new), H->d + 1);
      EXPLORED[H_new->C] = H_new;
      if (H_goal == nullptr || H_new->f < H_goal->f) OPEN.push(H_new);
    }
  }

  // backtrack
  if (H_goal != nullptr) {
    auto H = H_goal;
    while (H != nullptr) {
      solution.push_back(H->C);
      H = H->parent;
    }
    std::reverse(solution.begin(), solution.end());
  }

  // print result
  if (H_goal != nullptr && OPEN.empty()) {
    solver_info(1, "solved optimally, objective: ", objective);
  } else if (H_goal != nullptr) {
    solver_info(1, "solved sub-optimally, objective: ", objective);
  } else if (OPEN.empty()) {
    solver_info(1, "no solution");
  } else {
    solver_info(1, "timeout");
  }

  // logging
  additional_info +=
      "optimal=" + std::to_string(H_goal != nullptr && OPEN.empty()) + "\n";
  additional_info += "objective=" + std::to_string(objective) + "\n";
  additional_info += "loop_cnt=" + std::to_string(loop_cnt) + "\n";
  additional_info += "num_node_gen=" + std::to_string(EXPLORED.size()) + "\n";

  // memory management
  for (auto a : A) delete a;
  for (auto itr : EXPLORED) delete itr.second;

  return solution;
}

void Planner::rewrite(HNode* H_from, HNode* H_to, HNode* H_goal,
                      std::stack<HNode*>& OPEN)
{
  // update neighbors
  H_from->neighbor.insert(H_to);

  // Dijkstra update
  std::queue<HNode*> Q({H_from});  // queue is sufficient
  while (!Q.empty()) {
    auto n_from = Q.front();
    Q.pop();
    for (auto n_to : n_from->neighbor) {
      auto g_val = n_from->g + get_edge_cost(n_from->C, n_to->C);
      if (g_val < n_to->g) {
        if (n_to == H_goal)
          solver_info(1, "cost update: ", n_to->g, " -> ", g_val);
        n_to->g = g_val;
        n_to->f = n_to->g + n_to->h;
        n_to->parent = n_from;
        n_to->d = n_from->d + 1;
        Q.push(n_to);
        if (H_goal != nullptr && n_to->f < H_goal->f) OPEN.push(n_to);
      }
    }
  }
}

int Planner::get_edge_cost(const Config& C1, const Config& C2)
{
  if (objective == OBJ_SUM_OF_LOSS) {
    int cost = 0;
    for (uint i = 0; i < N; ++i) {
      if (C1[i] != ins->goals[i] || C2[i] != ins->goals[i]) {
        cost += 1;
      }
    }
    return cost;
  }

  // default: makespan
  return 1;
}

int Planner::get_edge_cost(HNode* H_from, HNode* H_to)
{
  return get_edge_cost(H_from->C, H_to->C);
}

int Planner::get_h_value(const Config& C)
{
  int cost = 0;
  if (objective == OBJ_MAKESPAN) {
    for (auto i = 0; i < N; ++i) cost = std::max(cost, HT->get(C[i]->index, ins->goals[i]->index));
  } else if (objective == OBJ_SUM_OF_LOSS) {
    for (auto i = 0; i < N; ++i) cost += HT->get(C[i]->index, ins->goals[i]->index);
  }
  return cost;
}

void Planner::expand_lowlevel_tree(HNode* H, LNode* L)
{
  if (L->depth >= N) return;
  const auto i = H->order[L->depth];
  auto C = H->C[i]->neighbor;
  C.push_back(H->C[i]);
  // randomize
  if (MT != nullptr) std::shuffle(C.begin(), C.end(), *MT);
  // insert
  for (auto v : C) H->search_tree.push(new LNode(L, i, v));
}

bool Planner::get_new_config(HNode* H, LNode* L)
{
  // setup cache
  for (auto a : A) {
    // clear previous cache
    if (a->v_now != nullptr && occupied_now[a->v_now->id] == a) {
      occupied_now[a->v_now->id] = nullptr;
    }
    if (a->v_next != nullptr) {
      occupied_next[a->v_next->id] = nullptr;
      a->v_next = nullptr;
    }

    // set occupied now
    a->v_now = H->C[a->id];
    occupied_now[a->v_now->id] = a;
  }

  // for (int i=0;i<occupied_next.size();++i){
  //   if (occupied_next[i]!=nullptr) {
  //     cerr<<"occupied_next is not empty"<<endl;
  //     exit(-1);
  //   }
  // }

  // add constraints
  for (uint k = 0; k < L->depth; ++k) {
    const auto i = L->who[k];        // agent
    const auto l = L->where[k]->id;  // loc

    // check vertex collision
    if (occupied_next[l] != nullptr){
      cerr<<"vertex collision"<<endl;
      exit(-1);
      return false;
    }
    // check swap collision
    auto l_pre = H->C[i]->id;
    if (occupied_next[l_pre] != nullptr && occupied_now[l] != nullptr &&
        occupied_next[l_pre]->id == occupied_now[l]->id) {
          cerr<<"swap collision"<<endl;
          exit(-1);
      return false;
    }

    // set occupied_next
    A[i]->v_next = L->where[k];
    occupied_next[l] = A[i];
  }

  // perform PIBT
  for (auto k : H->order) {
    auto a = A[k];
    if (a->v_next == nullptr && !funcPIBT(a,H)){
      // cerr<<"planning failture"<<endl;
      // exit(-1);
      return false;  // planning failure
    } 
  }
  return true;
}

int get_neighbor_orientation(const Graph & G, int loc1, int loc2) {

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

    if (loc1==loc2) {
      return 5;
    }

    std::cerr<<"loc1 and loc2 are not neighbors: "<<loc1<<", "<<loc2<<endl;
    exit(-1);

}

bool Planner::funcPIBT(Agent* ai, HNode * H)
{
  const auto i = ai->id;
  const auto K = ai->v_now->neighbor.size();

  // get candidates for next locations
  for (auto k = 0; k < K; ++k) {
    auto u = ai->v_now->neighbor[k];
    C_next[i][k] = u;
    if (MT != nullptr)
      tie_breakers[u->id] = get_random_float(MT);  // set tie-breaker
  }
  C_next[i][K] = ai->v_now;
  tie_breakers[ai->v_now->id] = get_random_float(MT);  // set tie-breaker

  // for (int j=0;j<K+1;++j){
  //     std::cerr<<"check C_next "<<j<<" "<<K<<endl;
  //     std::cerr<<C_next[i][j]<<endl;
  // }

  // sort
  std::sort(C_next[i].begin(), C_next[i].begin() + K + 1,
            [&](Vertex* const v, Vertex* const u) {

    // cerr<<"sort "<<v<<" "<<u<<endl;


    int o1=get_neighbor_orientation(ins->G,ai->v_now->index,v->index);
    int o2=get_neighbor_orientation(ins->G,ai->v_now->index,u->index);

    double d1,d2;
    if (use_orient_in_heuristic){
      d1=HT->get(v->index,o1,ins->goals[i]->index);
      d2=HT->get(u->index,o2,ins->goals[i]->index);      
    } else {
      d1=HT->get(v->index,ins->goals[i]->index);
      d2=HT->get(u->index,ins->goals[i]->index);
    }

    // if (ins->precomputed_paths!=nullptr){
    //   auto path=(*ins->precomputed_paths)[i];
    //   // for (int j=path.size()-1;j>=0;--j){
    //     int j=H->d;
    //     if (j<path.size()-1 && path[j].location==ai->v_now->index) {
    //       if (path[j+1].location==v->index) {
    //         d1=0.1;
    //         // break;
    //       }
    //       if (path[j+1].location==u->index) {
    //         d2=0.1;
    //         // break;
    //       }
    //     }
    //   // }
    // }

    if (ins->precomputed_paths!=nullptr){
      auto path=(*ins->precomputed_paths)[i];
      for (int j=path.size()-1;j>=0;--j){
        // int j=H->d;
        if (j<path.size()-1 && path[j].location==ai->v_now->index) {
          if (path[j+1].location==v->index) {
            d1=0.1;
            // break;
          }
          if (path[j+1].location==u->index) {
            d2=0.1;
            // break;
          }
        }
      }
    }



    if (d1!=d2) return d1<d2;

    if (MC_idx==0){
      return o1<o2;
    } else {
      return tie_breakers[v->id] < tie_breakers[u->id];
    }
    return o1<o2;

  });

  Agent* swap_agent=nullptr;
  if (use_swap) {
    swap_agent = swap_possible_and_required(ai);
    if (swap_agent != nullptr)
      std::reverse(C_next[i].begin(), C_next[i].begin() + K + 1);
  }

  // main operation
  for (auto k = 0; k < K + 1; ++k) {
    auto u = C_next[i][k];

    // avoid vertex conflicts
    if (occupied_next[u->id] != nullptr) continue;

    auto& ak = occupied_now[u->id];

    // avoid swap conflicts
    if (ak != nullptr && ak->v_next == ai->v_now) continue;

    // reserve next location
    occupied_next[u->id] = ai;
    ai->v_next = u;

    // priority inheritance
    if (ak != nullptr && ak != ai && ak->v_next == nullptr && !funcPIBT(ak,H))
      continue;

    // success to plan next one step
    // pull swap_agent when applicable
    if (use_swap) {
      if (k == 0 && swap_agent != nullptr && swap_agent->v_next == nullptr &&
          occupied_next[ai->v_now->id] == nullptr) {
        swap_agent->v_next = ai->v_now;
        occupied_next[swap_agent->v_next->id] = swap_agent;
      }
    }
    return true;
  }

  // failed to secure node
  occupied_next[ai->v_now->id] = ai;
  ai->v_next = ai->v_now;
  return false;
}

Agent* Planner::swap_possible_and_required(Agent* ai)
{
  const auto i = ai->id;
  // ai wanna stay at v_now -> no need to swap
  if (C_next[i][0] == ai->v_now) return nullptr;

  // usual swap situation, c.f., case-a, b
  auto aj = occupied_now[C_next[i][0]->id];
  if (aj != nullptr && aj->v_next == nullptr &&
      is_swap_required(ai->id, aj->id, ai->v_now, aj->v_now) &&
      is_swap_possible(aj->v_now, ai->v_now)) {
    return aj;
  }

  // for clear operation, c.f., case-c
  for (auto u : ai->v_now->neighbor) {
    auto ak = occupied_now[u->id];
    if (ak == nullptr || C_next[i][0] == ak->v_now) continue;
    if (is_swap_required(ak->id, ai->id, ai->v_now, C_next[i][0]) &&
        is_swap_possible(C_next[i][0], ai->v_now)) {
      return ak;
    }
  }

  return nullptr;
}

// simulate whether the swap is required
bool Planner::is_swap_required(const uint pusher, const uint puller,
                               Vertex* v_pusher_origin, Vertex* v_puller_origin)
{
  auto v_pusher = v_pusher_origin;
  auto v_puller = v_puller_origin;
  Vertex* tmp = nullptr;
  while (
      HT->get(v_puller->index, ins->goals[pusher]->index) < HT->get(v_pusher->index, ins->goals[pusher]->index)
    ) {
    auto n = v_puller->neighbor.size();
    // remove agents who need not to move
    for (auto u : v_puller->neighbor) {
      auto a = occupied_now[u->id];
      if (u == v_pusher ||
          (u->neighbor.size() == 1 && a != nullptr && ins->goals[a->id] == u)) {
        --n;
      } else {
        tmp = u;
      }
    }
    if (n >= 2) return false;  // able to swap
    if (n <= 0) break;
    v_pusher = v_puller;
    v_puller = tmp;
  }

  // judge based on distance
  return (HT->get(v_pusher->index, ins->goals[puller]->index) < HT->get(v_puller->index, ins->goals[puller]->index)) &&
          (HT->get(v_pusher->index, ins->goals[pusher]->index) == 0 || HT->get(v_puller->index, ins->goals[pusher]->index) < HT->get(v_pusher->index, ins->goals[pusher]->index));
}

// simulate whether the swap is possible
bool Planner::is_swap_possible(Vertex* v_pusher_origin, Vertex* v_puller_origin)
{
  auto v_pusher = v_pusher_origin;
  auto v_puller = v_puller_origin;
  Vertex* tmp = nullptr;
  while (v_puller != v_pusher_origin) {  // avoid loop
    auto n = v_puller->neighbor.size();  // count #(possible locations) to pull
    for (auto u : v_puller->neighbor) {
      auto a = occupied_now[u->id];
      if (u == v_pusher ||
          (u->neighbor.size() == 1 && a != nullptr && ins->goals[a->id] == u)) {
        --n;      // pull-impossible with u
      } else {
        tmp = u;  // pull-possible with u
      }
    }
    if (n >= 2) return true;  // able to swap
    if (n <= 0) return false;
    v_pusher = v_puller;
    v_puller = tmp;
  }
  return false;
}

std::ostream& operator<<(std::ostream& os, const Objective obj)
{
  if (obj == OBJ_NONE) {
    os << "none";
  } else if (obj == OBJ_MAKESPAN) {
    os << "makespan";
  } else if (obj == OBJ_SUM_OF_LOSS) {
    os << "sum_of_loss";
  }
  return os;
}

}  // namespace LaCAM2