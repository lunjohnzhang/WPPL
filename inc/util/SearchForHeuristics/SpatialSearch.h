#pragma once
#include "util/SearchForHeuristics/SpatialState.h"
#include "util/SearchForHeuristics/DataStructure.h"
#include "SharedEnv.h"
#include "util/MyLogger.h"

namespace RIVERS {

namespace SPATIAL {

class SpatialAStar {

public:
    SpatialAStar(
        const SharedEnvironment & env, int n_orients, const std::vector<int> & weights
    ): env(env), n_orients(n_orients), weights(weights) {
        max_states=env.rows*env.cols*n_orients;
        n_states=0;
        open_list = new OpenList(max_states);
        all_states = new State[max_states];
        max_successors=8;
        successors = new State[max_successors];
        reset();
    };

    void reset() {
        open_list->clear();
        n_states=0;
        for (int i=0;i<max_states;++i) {
            if (all_states[i].pos!=-1) {
                all_states[i].pos=-1;
                all_states[i].orient=-1;
                all_states[i].g=-1;
                all_states[i].h=0;
                all_states[i].f=-1;
                all_states[i].prev=nullptr;
            }
        }
        n_successors=0;
    }

    ~SpatialAStar() {
        delete open_list;
        delete [] all_states;
        delete [] successors;
    }
    
    int n_orients;
    int n_states;
    int max_states;
    OpenList* open_list;
    State * all_states;
    const int n_dirs=4;

    int n_successors;
    int max_successors;
    State * successors;
    
    const SharedEnvironment & env;
    const std::vector<int> & weights;

    void clear_successors() {
        n_successors=0;
    }

    void add_successor(int pos, int orient, int g, int h, State * prev) {
        // std::cerr<<"add successor: "<<pos<<" "<<orient<<" "<<g<<" "<<h<<std::endl;
        successors[n_successors].pos=pos;
        successors[n_successors].orient=orient;
        successors[n_successors].g=g;
        successors[n_successors].h=h;
        successors[n_successors].f=g+h;
        successors[n_successors].prev=prev;

        ++n_successors;
    }


    // currently no heuristic is used, so it is dijkstra actually.
    void get_successors(State * curr) {
        clear_successors();
        if (curr->orient==-1) {
            int pos=curr->pos;
            int x=pos%(env.cols);
            int y=pos/(env.cols);

            // east
            if (x+1<env.cols) {
                int next_pos=pos+1;
                if (env.map[next_pos]==0) {
                    int weight_idx=pos*n_dirs;
                    // std::cerr<<"east: "<<next_pos<<" "<<curr->g<<" "<<weights[weight_idx]<<endl;
                    add_successor(next_pos, -1, curr->g+weights[weight_idx], 0, curr);
                }
            }

            // south
            if (y+1<env.rows) {
                int next_pos=pos+env.cols;
                if (env.map[next_pos]==0) {
                    int weight_idx=pos*n_dirs+1;
                    // std::cerr<<"south: "<<next_pos<<" "<<curr->g<<" "<<weights[weight_idx]<<endl;
                    add_successor(next_pos, -1, curr->g+weights[weight_idx], 0, curr);
                }
            }

            // west
            if (x-1>=0) {
                int next_pos=pos-1;
                if (env.map[next_pos]==0) {
                    int weight_idx=pos*n_dirs+2;
                    // std::cerr<<"west: "<<next_pos<<" "<<curr->g<<" "<<weights[weight_idx]<<endl;
                    add_successor(next_pos, -1, curr->g+weights[weight_idx], 0, curr);
                }
            }

            // north
            if (y-1>=0) {
                int next_pos=pos-env.cols;
                if (env.map[next_pos]==0) {
                    int weight_idx=pos*n_dirs+3;
                    // std::cerr<<"north: "<<next_pos<<" "<<curr->g<<" "<<weights[weight_idx]<<endl;
                    add_successor(next_pos, -1, curr->g+weights[weight_idx], 0, curr);
                }
            }
        } else {
            g_logger.error("Spatial Search with orientation is not supported now!");
            exit(-1);
        }
    }

    State * add_state(int pos, int orient, int g, int h, State * prev) {
        int index=pos;
        State * s=all_states+index;
        if (s->pos!=-1) {
            g_logger.error("State already exists!");
            exit(-1);
        }

        s->pos=pos;
        s->orient=orient;
        s->g=g;
        s->h=h;
        s->f=g+h;
        s->prev=prev;

        ++n_states;
        return s;
    }


    void search_for_all(int start_pos, int start_orient=-1) {
        State * start=add_state(start_pos, start_orient, 0, 0, nullptr);
        open_list->push(start);

        while (!open_list->empty()) {
            State * curr=open_list->pop();
            curr->closed=true;

            get_successors(curr);
            for (int i=0;i<n_successors;++i) {
                State * next=successors+i;
                if ((all_states+next->pos)->pos==-1) {
                    // new state
                    State * new_state=add_state(next->pos, next->orient, next->g, next->h, next->prev);
                    new_state->closed=false;
                    open_list->push(new_state);
                } else {
                    // old state
                    auto old_state=all_states+next->pos;
                    if (next->g<old_state->g) {
                        // we need to update the state
                        old_state->copy(next);
                        if (old_state->closed) {
                            old_state->closed=false;
                            open_list->push(old_state);
                        } else {
                            open_list->increase(old_state);
                        }
                    }
                }
            }
        }
    }

};

}

}