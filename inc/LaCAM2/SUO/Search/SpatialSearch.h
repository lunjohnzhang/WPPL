#pragma once
#include "LaCAM2/SUO/Search/SpatialState.h"
#include "LaCAM2/SUO/Search/SpatialDataStructure.h"
#include "SharedEnv.h"
#include "util/MyLogger.h"
#include <vector>
#include "util/HeuristicTable.h"

namespace SUO {

namespace Spatial {

class AStar {

public:
    AStar(
        const SharedEnvironment & env, int n_orients, const std::vector<int> & weights, std::shared_ptr<HeuristicTable> & HT, int window
    ): env(env), n_orients(n_orients), weights(weights),cost_map(env.cols,env.rows), HT(HT), window(window) {
        max_states=env.rows*env.cols*n_orients;
        n_states=0;
        open_list = new OpenList(max_states);
        all_states = new State[max_states];
        max_successors=8;
        successors = new State[max_successors];

        reset_plan();
    };

    void reset_plan() {
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
        n_expanded=0;
        n_generated=0;
    }

    void clear_cost_map() {
        this->cost_map.clear();
    }

    void update_cost_map(const std::vector<std::pair<int,float> > & deltas) {
        // make a copy
        this->cost_map.update(deltas);
    }

    void remove_path_cost(const std::vector<State> & path, float vertex_collision_cost) {
        for (const auto & state: path) {
            this->cost_map[state.pos]-=vertex_collision_cost;
        }
    }

    void add_path_cost(const std::vector<State> & path, float vertex_collision_cost) {
        for (const auto & state: path) {
            this->cost_map[state.pos]+=vertex_collision_cost;
        }
    }

    ~AStar() {
        delete open_list;
        delete [] all_states;
        delete [] successors;
    }
    
    int n_expanded;
    int n_generated;

    int window;

    int n_orients;
    int n_states;
    int max_states;
    OpenList* open_list;
    State * all_states;
    const int n_dirs=5; // right, down, left, up, stay

    int n_successors;
    int max_successors;
    State * successors;
    
    const SharedEnvironment & env;
    const std::vector<int> & weights;
    std::shared_ptr<HeuristicTable> HT;
    CostMap cost_map;

    void clear_successors() {
        n_successors=0;
    }

    void add_successor(int pos, int orient, float g, float h, State * prev) {
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
    void get_successors(State * curr, int start_pos, int goal_pos) {
        clear_successors();
        if (curr->orient==-1) {
            int pos=curr->pos;
            int x=pos%(env.cols);
            int y=pos/(env.cols);

            // BUG(rivers): why consisten heuristic slows down the search? check if reopen 

            // east
            if (x+1<env.cols) {
                int next_pos=pos+1;
                if (env.map[next_pos]==0) {
                    int weight_idx=pos*n_dirs;
                    float collision_cost=cost_map[next_pos];
                    float h=HT->get(next_pos, goal_pos);
                    int d=HT->get(start_pos,next_pos);
                    if (d>20) {
                        collision_cost=0;
                    }
                    // std::cerr<<"east: "<<next_pos<<" "<<curr->g<<" "<<weights[weight_idx]<<endl;
                    //BUG(rivers): where is our heuristic?
                    add_successor(next_pos, -1, curr->g+weights[weight_idx]+collision_cost, h, curr);
                }
            }

            // south
            if (y+1<env.rows) {
                int next_pos=pos+env.cols;
                if (env.map[next_pos]==0) {
                    int weight_idx=pos*n_dirs+1;
                    float collision_cost=cost_map[next_pos];
                    float h=HT->get(next_pos, goal_pos);
                    int d=HT->get(start_pos,next_pos);
                    if (d>20) {
                        collision_cost=0;
                    }
                    // std::cerr<<"south: "<<next_pos<<" "<<curr->g<<" "<<weights[weight_idx]<<endl;
                    add_successor(next_pos, -1, curr->g+weights[weight_idx]+collision_cost, h, curr);
                }
            }

            // west
            if (x-1>=0) {
                int next_pos=pos-1;
                if (env.map[next_pos]==0) {
                    int weight_idx=pos*n_dirs+2;
                    float collision_cost=cost_map[next_pos];
                    float h=HT->get(next_pos, goal_pos);
                    int d=HT->get(start_pos,next_pos);
                    if (d>20) {
                        collision_cost=0;
                    }
                    // std::cerr<<"west: "<<next_pos<<" "<<curr->g<<" "<<weights[weight_idx]<<endl;
                    add_successor(next_pos, -1, curr->g+weights[weight_idx]+collision_cost, h, curr);
                }
            }

            // north
            if (y-1>=0) {
                int next_pos=pos-env.cols;
                if (env.map[next_pos]==0) {
                    int weight_idx=pos*n_dirs+3;
                    float collision_cost=cost_map[next_pos];
                    float h=HT->get(next_pos, goal_pos);
                    int d=HT->get(start_pos,next_pos);
                    if (d>20) {
                        collision_cost=0;
                    }
                    // std::cerr<<"north: "<<next_pos<<" "<<curr->g<<" "<<weights[weight_idx]<<endl;
                    add_successor(next_pos, -1, curr->g+weights[weight_idx]+collision_cost, h, curr);
                }
            }
        } else {
            g_logger.error("Spatial Search with orientation is not supported now!");
            exit(-1);
        }
    }

    State * add_state(int pos, int orient, float g, float h, State * prev) {
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

    State* search(int start_pos, int start_orient, int goal_pos) {
        State * start=add_state(start_pos, start_orient, 0, HT->get(start_pos, goal_pos), nullptr);
        open_list->push(start);

        while (!open_list->empty()) {
            State * curr=open_list->pop();
            curr->closed=true;
            ++n_expanded;

            // goal checking
            if (goal_pos==curr->pos) {
                // cerr<<n_expanded<<" "<<n_generated<<endl;
                // TODO return the path
                return curr;
            }

            get_successors(curr, start_pos, goal_pos);
            for (int i=0;i<n_successors;++i) {
                ++n_generated;
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
                            std::cerr<<"reopen"<<std::endl;
                            exit(-1);
                            old_state->closed=false;
                            open_list->push(old_state);
                        } else {
                            open_list->increase(old_state);
                        }
                    }
                }
            }
        }

        return nullptr;        
    }

};

}

}