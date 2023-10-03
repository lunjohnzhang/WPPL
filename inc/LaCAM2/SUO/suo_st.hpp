#pragma once
#include "LaCAM2/instance.hpp"
#include "common.h"
#include "util/HeuristicTable.h"
#include "States.h"
#include "boost/heap/pairing_heap.hpp"
#include <cstdlib>

namespace LaCAM2 {

class SUO {
public:
    struct State {
        Vertex * v;
        float g;
        float h;
        float f;
        State * prev;
    
        State(Vertex * v, float g, float h, State * prev): v(v), g(g), h(h), f(g+h), closed(false), prev(prev) {};
        State(Vertex *v): State(v,-1,0,nullptr){};

        void copy(const State * s) {
            v = s->v;
            g = s->g;
            h = s->h;
            f = s->f;
            prev = s->prev;
        }

        struct StateCompare {
            bool operator()(const State * s1, const State * s2) const {
                if (s1->f == s2->f){
                    if (s1->g == s2->g) {
                        return rand()%2==0;
                    }
                    return s1->g > s2->g;
                }
                return s1->f > s2->f;
            }
        };

        struct StateHash {
            std::size_t operator()(const State * s) const {
                size_t loc_hash = std::hash<int>()(s->v->id);
                return loc_hash;
            }
        };

        struct StateEqualTo {
            bool operator()(const State * s1, const State * s2) const {
                return s1->v->id == s2->v->id;
            }
        };

        typedef boost::heap::pairing_heap<State*,boost::heap::compare<State::StateCompare> >::handle_type open_handle_t;

        bool closed;
        open_handle_t open_handle;
    };

    boost::heap::pairing_heap<State*,boost::heap::compare<State::StateCompare> > open_list;
    boost::unordered_set<State*,State::StateHash,State::StateEqualTo> all_states;

    std::shared_ptr<Graph> G;
    std::shared_ptr<HeuristicTable> HT;
    std::vector<std::vector<State> > paths;
    std::vector<float> path_costs;
    std::vector<int> orders;

    std::vector<float> cost_map;
    float vertex_collision_cost;
    int iterations;
    int max_expanded;
    float h_weight;

    SUO(
        std::shared_ptr<Graph> & G, 
        std::shared_ptr<HeuristicTable> & HT,
        float vertex_collision_cost,
        int iterations,
        int max_expanded,
        float h_weight
    ): G(G), HT(HT), vertex_collision_cost(vertex_collision_cost), 
    iterations(iterations), max_expanded(max_expanded), h_weight(h_weight) {};

    void plan(Instance & instance);
    void init(Instance & instance);

    // TODO: we may replace it with incremental A* later
    void AStar(int agent_idx, Vertex * start, Vertex * goal);
    void generate_and_insert_next_state(State * curr_state, Vertex * next, Vertex * goal);
    void update_path(int agent_idx, State * goal_state);
};

} // namespace LaCAM2