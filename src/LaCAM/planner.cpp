#include "LaCAM/planner.h"

namespace LaCAM {

Constraint::Constraint() : who(std::vector<int>()), where(Vertices()), depth(0)
{
}

Constraint::Constraint(Constraint* parent, int i, Vertex* v)
    : who(parent->who), where(parent->where), depth(parent->depth + 1)
{
  who.push_back(i);
  where.push_back(v);
}

Constraint::~Constraint(){};

Node::Node(Config _C, const std::shared_ptr<HeuristicTable> & H, const Instance * ins, Node* _parent)
    : C(_C),
      ins(ins),
      parent(_parent),
      priorities(C.size(), 0),
      order(C.size(), 0),
      search_tree(std::queue<Constraint*>())
{
  search_tree.push(new Constraint());
  const auto N = C.size();

  // set priorities
  // if (parent == nullptr) {
  //   // initialize
  //   for (size_t i = 0; i < N; ++i) priorities[i] = (float)(H->get(C[i]->index,ins->goals[i]->index))/N;
  // } else {
  //   // dynamic priorities, akin to PIBT
  //   for (size_t i = 0; i < N; ++i) {
  //     if (C[i]->index != ins->goals[i]->index) {
  //       priorities[i] = parent->priorities[i] + 1;
  //     } else {
  //       priorities[i] = parent->priorities[i] - (int)parent->priorities[i];
  //     }
  //   }
  // }


  // for (size_t i = 0; i < N; ++i) {
  //   if (C[i]->index != ins->goals[i]->index) {
  //     priorities[i] = ins->priorities[i];
  //   } else {
  //     priorities[i] = 0;
  //   }
  // }

  // set order
  std::iota(order.begin(), order.end(), 0);
  std::sort(order.begin(), order.end(), [&](int i, int j) { 
        const AgentInfo & a=ins->agent_infos[i];
        const AgentInfo & b=ins->agent_infos[j];

        if (a.elapsed!=b.elapsed) return a.elapsed>b.elapsed;

        return a.tie_breaker>b.tie_breaker;
  });
  
  // for (int i=0;i<order.size();++i) {
  //   cerr<<i<<" "<<order[i]<<endl;
  // }


}

Node::~Node()
{
  while (!search_tree.empty()) {
    delete search_tree.front();
    search_tree.pop();
  }
}

Planner::Planner(const Instance* _ins, const std::shared_ptr<HeuristicTable> & H, const Deadline* _deadline,
                 std::mt19937* _MT, int _verbose)
    : ins(_ins),
      deadline(_deadline),
      MT(_MT),
      verbose(_verbose),
      N(ins->N),
      V_size(ins->G.size()),
      H(H),
      C_next(Candidates(N, std::array<Vertex*, 5>())),
      tie_breakers(std::vector<float>(V_size, 0)),
      A(Agents(N, nullptr)),
      occupied_now(Agents(V_size, nullptr)),
      occupied_next(Agents(V_size, nullptr))
{
}

// Planner::Planner(const Graph & G, int N, const Deadline* _deadline, std::mt19937* _MT, int _verbose=0):
//   ins(nullptr),
//   deadline(_deadline),
//   MT(_MT),
//   verbose(_verbose),
//   N(N),
//   V_size(G.size()),
//   D(), // TODO
//   C_next(Candidates(N, std::array<Vertex*, 5>())),
//   tie_breakers(std::vector<float>(V_size, 0)),
//   A(Agents(N, nullptr)),
//   occupied_now(Agents(V_size, nullptr)),
//   occupied_next(Agents(V_size, nullptr))
// {
// }

Solution Planner::solve()
{
  info(1, verbose, "elapsed:", elapsed_ms(deadline), "ms\tstart search");

  // setup agents
  for (auto i = 0; i < N; ++i) A[i] = new Agent(i);

  // setup search queues
  std::stack<Node*> OPEN;
  std::unordered_map<Config, Node*, ConfigHasher> CLOSED;
  std::vector<Constraint*> GC;  // garbage collection of constraints

  // insert initial node
  auto S = new Node(ins->starts, H, ins);
  OPEN.push(S);
  CLOSED[S->C] = S;

  // depth first search
  int loop_cnt = 0;
  std::vector<Config> solution;

  while (!OPEN.empty() && !is_expired(deadline)) {
    loop_cnt += 1;

    // do not pop here!
    S = OPEN.top();

    // check goal condition: only require someone to arrive in lifelong case
    bool someone_arrived = false;
    for (int i=0;i<N;++i) {
      if (S->C[i]->id == ins->goals[i]->id || loop_cnt>1){
        // backtrack
        while (S != nullptr) {
          solution.push_back(S->C);
          S = S->parent;
        }
        std::reverse(solution.begin(), solution.end());
        someone_arrived = true;
        break;
      }
    }

    if (someone_arrived || loop_cnt>1) break;

    // if (is_same_config(S->C, ins->goals)) {
    //   // backtrack
    //   while (S != nullptr) {
    //     solution.push_back(S->C);
    //     S = S->parent;
    //   }
    //   std::reverse(solution.begin(), solution.end());
    //   break;
    // }

    // low-level search end
    if (S->search_tree.empty()) {
      OPEN.pop();
      continue;
    }

    // create successors at the low-level search
    auto M = S->search_tree.front();
    GC.push_back(M);
    S->search_tree.pop();
    if (M->depth < N) {
      auto i = S->order[M->depth];
      auto C = S->C[i]->neighbor;
      C.push_back(S->C[i]);
      if (MT != nullptr) std::shuffle(C.begin(), C.end(), *MT);  // randomize
      for (auto u : C) S->search_tree.push(new Constraint(M, i, u));
    }

    // create successors at the high-level search
    if (!get_new_config(S, M)) continue;

    // create new configuration
    auto C = Config(N, nullptr);
    for (auto a : A) C[a->id] = a->v_next;

    // check explored list
    auto iter = CLOSED.find(C);
    if (iter != CLOSED.end()) {
      OPEN.push(iter->second);
      continue;
    }

    // insert new search node
    auto S_new = new Node(C, H, ins, S);
    OPEN.push(S_new);
    CLOSED[S_new->C] = S_new;
  }

  info(1, verbose, "elapsed:", elapsed_ms(deadline), "ms\t",
       solution.empty() ? (OPEN.empty() ? "no solution" : "failed")
                        : "solution found",
       "\tloop_itr:", loop_cnt, "\texplored:", CLOSED.size());
  // memory management
  for (auto a : A) delete a;
  for (auto M : GC) delete M;
  for (auto p : CLOSED) delete p.second;

  return solution;
}

bool Planner::get_new_config(Node* S, Constraint* M)
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
    a->v_now = S->C[a->id];
    occupied_now[a->v_now->id] = a;
  }

  // add constraints
  for (auto k = 0; k < M->depth; ++k) {
    const auto i = M->who[k];        // agent
    const auto l = M->where[k]->id;  // loc

    // check vertex collision
    if (occupied_next[l] != nullptr) return false;
    // check swap collision
    auto l_pre = S->C[i]->id;
    if (occupied_next[l_pre] != nullptr && occupied_now[l] != nullptr &&
        occupied_next[l_pre]->id == occupied_now[l]->id)
      return false;

    // set occupied_next
    A[i]->v_next = M->where[k];
    occupied_next[l] = A[i];
  }

  // perform PIBT
  for (auto k : S->order) {
    auto a = A[k];
    if (a->v_next == nullptr && !funcPIBT(a)) return false;  // planning failure
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

bool Planner::funcPIBT(Agent* ai)
{
  const auto i = ai->id;
  const auto K = ai->v_now->neighbor.size();

  // get candidates for next locations
  for (size_t k = 0; k < K; ++k) {
    auto u = ai->v_now->neighbor[k];
    C_next[i][k] = u;
    if (MT != nullptr)
      tie_breakers[u->id] = get_random_float(MT);  // set tie-breaker
  }
  C_next[i][K] = ai->v_now;

  // sort, note: K + 1 is sufficient
  std::sort(C_next[i].begin(), C_next[i].begin() + K + 1, [&](Vertex* const v, Vertex* const u) {


    double d1=H->get(v->index,ins->goals[i]->index);
    double d2=H->get(u->index,ins->goals[i]->index);

    if (d1!=d2) return d1<d2;

    int o1=get_neighbor_orientation(ins->G,ai->v_now->index,v->index);
    int o2=get_neighbor_orientation(ins->G,ai->v_now->index,u->index);

    return o1<o2;


    return false;
  });

  for (size_t k = 0; k < K + 1; ++k) {
    auto u = C_next[i][k];

    // avoid vertex conflicts
    if (occupied_next[u->id] != nullptr) continue;

    auto& ak = occupied_now[u->id];

    // avoid swap conflicts with constraints
    if (ak != nullptr && ak->v_next == ai->v_now) continue;

    // reserve next location
    occupied_next[u->id] = ai;
    ai->v_next = u;

    // empty or stay
    if (ak == nullptr || u == ai->v_now) return true;

    // priority inheritance
    if (ak->v_next == nullptr && !funcPIBT(ak)) continue;

    // success to plan next one step
    return true;
  }

  // failed to secure node
  occupied_next[ai->v_now->id] = ai;
  ai->v_next = ai->v_now;
  return false;
}

Solution solve(const Instance& ins, const std::shared_ptr<HeuristicTable> & H, int verbose, const Deadline* deadline,
               std::mt19937* MT)
{
  info(1, verbose, "elapsed:", elapsed_ms(deadline), "ms\tpre-processing");
  auto planner = Planner(&ins, H, deadline, MT, verbose);
  return planner.solve();
}

}