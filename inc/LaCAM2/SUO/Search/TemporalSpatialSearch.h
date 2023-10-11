#pragma once
#include "LaCAM2/SUO/Search/TemporalSpatialState.h"
#include "SharedEnv.h"
#include "util/MyLogger.h"
#include <vector>
#include "boost/heap/pairing_heap.hpp"
#include "boost/unordered_set.hpp"
#include "LaCAM2/SUO/Search/TemporalSpatialDataStructure.h"
#include "util/HeuristicTable.h"

namespace SUO {

namespace TemporalSpatial {

class AStar {

public:
    AStar(
        const SharedEnvironment & env, int n_orients, const std::vector<int> & weights, std::shared_ptr<HeuristicTable> & HT, int window
    ): env(env), n_orients(n_orients), weights(weights), cost_map(env.cols,env.rows,window+1), HT(HT), window(window) {
        reset_plan();
    };

    void reset_plan() {
        open_list.clear();
        for (auto & state: all_states) {
            delete state;
        }
        all_states.clear();
        n_expanded=0;
        n_generated=0;
    }

    void clear_cost_map() {
        this->cost_map.clear();
    }

    void update_cost_map(const std::vector<std::pair<State,float> > & deltas) {
        // make a copy
        this->cost_map.update(deltas);
    }

    void remove_path_cost(const std::vector<State> & path, float vertex_collision_cost) {
        for (const auto & state: path) {
            this->cost_map(state.pos,state.t)-=vertex_collision_cost;
        }
    }

    void add_path_cost(const std::vector<State> & path, float vertex_collision_cost) {
        for (const auto & state: path) {
            this->cost_map(state.pos,state.t)+=vertex_collision_cost;
        }
    }


    ~AStar() {

    }

    int n_expanded;
    int n_generated;

    int window;
    
    int n_orients;    
    const int n_dirs=5; // right, down, left, up, stay
    boost::heap::pairing_heap<State*, boost::heap::compare<State::StateCompare> > open_list;
    boost::unordered_set<State*, State::StateHash, State::StateEqual> all_states;
    std::vector<State *> successors;
    
    const SharedEnvironment & env;
    const std::vector<int> & weights;
    std::shared_ptr<HeuristicTable> HT;
    CostMap cost_map;

    // currently no heuristic is used, so it is dijkstra actually.
    void get_successors(State * curr, int start_pos, int goal_pos) {
        successors.clear();
        if (curr->orient==-1) {
            int pos=curr->pos;
            int x=pos%(env.cols);
            int y=pos/(env.cols);

            int next_t=curr->t+1;

            // east
            if (x+1<env.cols) {
                int next_pos=pos+1;
                if (env.map[next_pos]==0) {
                    int weight_idx=pos*n_dirs;
                    float collision_cost=cost_map(next_pos,next_t);
                    float h=HT->get(next_pos, goal_pos);
                    // std::cerr<<"east: "<<next_pos<<" "<<curr->g<<" "<<weights[weight_idx]<<endl;
                    successors.push_back(new State(next_pos, -1, next_t, curr->g+weights[weight_idx]+collision_cost, h, curr));
                }
            }

            // south
            if (y+1<env.rows) {
                int next_pos=pos+env.cols;
                if (env.map[next_pos]==0) {
                    int weight_idx=pos*n_dirs+1;
                    float collision_cost=cost_map(next_pos,next_t);
                    float h=HT->get(next_pos, goal_pos);
                    // std::cerr<<"south: "<<next_pos<<" "<<curr->g<<" "<<weights[weight_idx]<<endl;
                    successors.push_back(new State(next_pos, -1, next_t, curr->g+weights[weight_idx]+collision_cost, h, curr));
                }
            }

            // west
            if (x-1>=0) {
                int next_pos=pos-1;
                if (env.map[next_pos]==0) {
                    int weight_idx=pos*n_dirs+2;
                    float collision_cost=cost_map(next_pos,next_t);
                    float h=HT->get(next_pos, goal_pos);
                    // std::cerr<<"west: "<<next_pos<<" "<<curr->g<<" "<<weights[weight_idx]<<endl;
                    successors.push_back(new State(next_pos, -1, next_t, curr->g+weights[weight_idx]+collision_cost, h, curr));
                }
            }

            // north
            if (y-1>=0) {
                int next_pos=pos-env.cols;
                if (env.map[next_pos]==0) {
                    int weight_idx=pos*n_dirs+3;
                    float collision_cost=cost_map(next_pos,next_t);
                    float h=HT->get(next_pos, goal_pos);
                    // std::cerr<<"north: "<<next_pos<<" "<<curr->g<<" "<<weights[weight_idx]<<endl;
                    successors.push_back(new State(next_pos, -1, next_t, curr->g+weights[weight_idx]+collision_cost, h, curr));
                }
            }

            // wait
            int next_pos=pos;
            // BUG(rivers): it is a bug we need to fix, the weight need to be read from file
            int weight_idx=pos*n_dirs+4;
            float collision_cost=cost_map(next_pos,next_t);
            float h=HT->get(next_pos, goal_pos);
            successors.push_back(new State(next_pos, -1, next_t, curr->g+weights[weight_idx]+collision_cost, h, curr));

        } else {
            DEV_ERROR("Temporal-Spatial Search with orientation is not supported now!");
            exit(-1);
        }
    }

    State* search(int start_pos, int start_orient, int goal_pos) {
        // DEV_DEBUG("start search from {} to {} with window {}", start_pos, goal_pos, window);

        State * start=new State(start_pos, -1, 0, 0, 0, nullptr);
        all_states.insert(start);
        start->closed=false;
        start->open_list_handle=open_list.push(start);

        while (!open_list.empty()) {
            State * curr=open_list.top();
            // std::cerr<<curr->pos<<" "<<curr->t<<" "<<curr->g<<" "<<curr->f<<endl;

            open_list.pop();
            curr->closed=true;
            ++n_expanded;

            // goal checking
            if (goal_pos==curr->pos) {
                // TODO return the path
                return curr;
            }

            if (curr->t>=window) {
                // TODO return the path
                return curr;
            }

            get_successors(curr,start_pos,goal_pos);
            for (auto & next_state: successors) {
                ++n_generated;
                auto iter = all_states.find(next_state);
                if (iter==all_states.end()) {
                    // new state
                    all_states.insert(next_state);
                    next_state->closed=false;
                    next_state->open_list_handle=open_list.push(next_state);
                } else {
                    // old state
                    auto old_state=(*iter);
                    if (next_state->g<old_state->g) {
                        // we need to update the state
                        old_state->copy(next_state);
                        if (old_state->closed) {
                            std::cerr<<"reopen"<<std::endl;
                            exit(-1);
                            // reopen closed state
                            old_state->closed=false;
                            old_state->open_list_handle=open_list.push(old_state);
                        } else {
                            // update open state
                            open_list.increase(old_state->open_list_handle);
                        }
                    }
                    delete next_state;
                }
            }
        }

        return nullptr;        
    }

};

}

}