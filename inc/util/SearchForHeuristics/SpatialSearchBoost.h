#pragma once
#include "util/SearchForHeuristics/SpatialState.h"
#include "util/SearchForHeuristics/DataStructure.h"
#include "SharedEnv.h"
#include "util/MyLogger.h"

namespace RIVERS {

namespace SPATIAL {

class SpatialAStarBoost {

public:
    SpatialAStarBoost(
        const SharedEnvironment & env, int n_orients, const std::vector<int> & weights
    ): env(env), n_orients(n_orients), weights(weights) {
        reset();
    };

    void reset() {
        open_list.clear();
        for (auto& state: all_states) {
            delete state;
        }
        all_states.clear();
    }

    ~SpatialAStarBoost() {
    }
    
    int n_orients;
    boost::heap::pairing_heap<State*, boost::heap::compare<State::StateCompare> > open_list;
    boost::unordered_set<State*, State::StateHash, State::StateEqual> all_states;
    const int n_dirs=4;

    int n_successors;
    std::vector<State*> successors;
    
    const SharedEnvironment & env;
    const std::vector<int> & weights;

    // currently no heuristic is used, so it is dijkstra actually.
    void get_successors(State * curr) {
        successors.clear();
        if (curr->orient==-1) {
            int pos=curr->pos;
            int x=pos%(env.cols);
            int y=pos/(env.cols);

            // east
            if (x+1<env.cols) {
                int next_pos=pos+1;
                if (env.map[next_pos]==0) {
                    int weight_idx=pos*n_dirs;
                    successors.emplace_back(new State(next_pos, -1, curr->g+weights[weight_idx], 0, curr));
                }
            }

            // south
            if (y+1<env.rows) {
                int next_pos=pos+env.cols;
                if (env.map[next_pos]==0) {
                    int weight_idx=pos*n_dirs+1;
                    successors.emplace_back(new State(next_pos, -1, curr->g+weights[weight_idx], 0, curr));
                }
            }

            // west
            if (x-1>=0) {
                int next_pos=pos-1;
                if (env.map[next_pos]==0) {
                    int weight_idx=pos*n_dirs+2;
                    successors.emplace_back(new State(next_pos, -1, curr->g+weights[weight_idx], 0, curr));
                }
            }

            // north
            if (y-1>=0) {
                int next_pos=pos-env.cols;
                if (env.map[next_pos]==0) {
                    int weight_idx=pos*n_dirs+3;
                    successors.emplace_back(new State(next_pos, -1, curr->g+weights[weight_idx], 0, curr));
                }
            }
        } else {
            DEV_ERROR("Spatial Search with orientation is not supported now!");
            exit(-1);
        }
    }

    void search_for_all(int start_pos, int start_orient=-1) {

        State * start=new State(start_pos, start_orient, 0, 0, nullptr);
        all_states.insert(start);
        start->closed=false;
        start->open_list_handle=open_list.push(start);

        while (!open_list.empty()) {
            State * curr=open_list.top();
            open_list.pop();
            curr->closed=true;
            // std::cerr<<curr->pos<<" "<<curr->g<<" "<<curr->h<<" "<<curr->f<<std::endl;

            get_successors(curr);
            for (auto & next_state: successors) {
                auto iter=all_states.find(next_state);
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
    }

};

}

}