#pragma once

#include "Routing/Region.h"
#include "util/HeuristicTable.h"
#include "common.h"
#include <unordered_set>
#include "Routing/State.h"
#include "SharedEnv.h"

namespace Routing {


struct AgentController
{
    int id;

    int curr_region=-1;
    int next_region_to_go_idx=-1;

    std::vector<int> region_path;

    std::shared_ptr<HeuristicTable> HT;
    std::shared_ptr<vector<float> > weights;
    const SharedEnvironment & env;
    
    std::vector<std::shared_ptr<Region> > & regions;

    boost::heap::pairing_heap<State*, boost::heap::compare<State::Compare> > open_list;
    boost::unordered_set<State*, State::Hash, State::Equal> all_states;
    std::vector<State*> successors;

    std::vector<std::pair<int, int> > path;

    AgentController(
        int id, 
        std::vector<std::shared_ptr<Region> > & regions, 
        std::shared_ptr<HeuristicTable> HT, 
        std::shared_ptr<vector<float> > weights,
        const SharedEnvironment & env
    ):
        id(id), 
        regions(regions), 
        HT(HT), 
        weights(weights),
        env(env) {
        
    }

    void reset() {
        curr_region=-1;
        next_region_to_go_idx=-1;
        region_path.clear();
    } 

    void clear() {
        open_list.clear();
        for (auto s: all_states) {
            delete s;
        }
        all_states.clear();
        path.clear();
    }

    void buildPath(State * curr) {
        path.push_back(std::make_pair(curr->pos, curr->orient));
        while (curr->prev!=nullptr) {
            curr=curr->prev;
            path.push_back(std::make_pair(curr->pos, curr->orient));
        }
        std::reverse(path.begin(), path.end());
    }

    // local routing: guide agent to the next region
    void route(int start_pos, int start_orient, int goal_pos) {
        clear();

        if (next_region_to_go_idx==-1) {
            std::cout<<"AgentController::route: next_region_to_go_idx==-1"<<std::endl;
            exit(-1);
        }

        auto & next_region_to_go=regions[next_region_to_go_idx];

        std::unordered_set<int> & local_goals=next_region_to_go->boundary;

        State * start_state = new State(start_pos, start_orient, 0, HT->get(start_pos, start_orient, goal_pos), -1, nullptr);
        all_states.insert(start_state);
        start_state->closed=false;
        start_state->open_list_handle=open_list.push(start_state);

        while (!open_list.empty()) {
            State * curr=open_list.top();
            open_list.pop();
            curr->closed=true;

            if (local_goals.find(curr->pos)!=local_goals.end()) {
                // we have found a path
                buildPath(curr);
                return;
            }

            getSuccessors(curr, goal_pos);
            for (auto & next_state: successors) {
                auto iter = all_states.find(next_state);
                if (iter==all_states.end()) {
                    // new state
                    all_states.insert(next_state);
                    next_state->closed=false;
                    next_state->open_list_handle=open_list.push(next_state);
                } else {
                    auto old_state=(*iter);
                    if (
                        next_state->f<old_state->f || (
                            next_state->f==old_state->f
                            && next_state->h<old_state->h
                        )
                    ) {
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

    void getSuccessors(State * curr, int goal_pos) {
        successors.clear();

        int cols=env.cols;
        int rows=env.rows;
        auto & map=env.map;
        auto & weights=*this->weights;
        int pos=curr->pos;
        int orient=curr->orient;
        int x=pos%cols;
        int y=pos/cols;
        int n_dirs=5; // include wait
        int n_orients=4; // east, south, west, north

        // FW
        int next_pos;
        int weight_idx;
        int next_orient=orient;
        float next_g;
        float next_h;
        if (orient==0) {
            // east
            if (x+1<cols){
                next_pos=pos+1;
                weight_idx=pos*n_dirs;
                if (map[next_pos]==0) {
                    next_g=curr->g+weights[weight_idx];
                    next_h=HT->get(next_pos, next_orient, goal_pos);
                    successors.push_back(new State(next_pos, next_orient, next_g, next_h, -1, curr));
                }
            }
        } else if (orient==1) {
            // south
            if (y+1<rows) {
                next_pos=pos+cols;
                weight_idx=pos*n_dirs+1;
                if (map[next_pos]==0) {
                    next_g=curr->g+weights[weight_idx];
                    next_h=HT->get(next_pos, next_orient, goal_pos);
                    successors.push_back(new State(next_pos, next_orient, next_g, next_h, -1, curr));
                }
            }
        } else if (orient==2) {
            // west
            if (x-1>=0) {
                next_pos=pos-1;
                weight_idx=pos*n_dirs+2;
                if (map[next_pos]==0) {
                    next_g=curr->g+weights[weight_idx];
                    next_h=HT->get(next_pos, next_orient, goal_pos);
                    successors.push_back(new State(next_pos, next_orient, next_g, next_h, -1, curr));
                }
            }
        } else if (orient==3) {
            // north
            if (y-1>=0) {
                next_pos=pos-cols;
                weight_idx=pos*n_dirs+3;
                if (map[next_pos]==0) {
                    next_g=curr->g+weights[weight_idx];
                    next_h=HT->get(next_pos, next_orient, goal_pos);
                    successors.push_back(new State(next_pos, next_orient, next_g, next_h, -1, curr));
                }
            }
        } else {
            std::cerr<<"TimeSpaceAStartPlanner: invalid orient: "<<orient<<endl;
            exit(-1);
        }


        // for actions that don't change the position
        next_pos=pos;

        weight_idx=pos*n_dirs+4;
        next_g=curr->g+weights[weight_idx];

        // CR
        next_orient=(orient+1+n_orients)%n_orients;
        next_h=HT->get(next_pos, next_orient, goal_pos);
        successors.push_back(new State(next_pos, next_orient, next_g, next_h, -1, curr));

        // CCR
        next_orient=(orient-1+n_orients)%n_orients;
        next_h=HT->get(next_pos, next_orient, goal_pos);
        successors.push_back(new State(next_pos, next_orient, next_g, next_h, -1, curr));

        // we don't have wait if without t.
        // W
        // next_orient=orient;
        // next_h=HT->get(next_pos, next_orient, goal_pos);
        // successors.push_back(new State(next_pos, next_orient, next_g, next_h, -1, curr));

    }



};

}