#pragma once

#include "util/CompetitionActionModel.h"
#include <random>
#include "graph.h"
#include "instance.h"
#include "planner.h"
#include <memory>
#include "util/HeuristicTable.h"
#include "LaCAM/executor.h"

namespace LaCAM {

class LaCAMSolver {

public:
    // [start] the following lines should be abstracted away.
    std::vector<Path> paths;
    CompetitionActionModelWithRotate action_model;
    std::mt19937* MT;   // seed for randomness
    bool need_replan = true;
    int total_feasible_timestep = 0;
    int timestep = 0;
    void initialize(const SharedEnvironment & env);
    void plan(const SharedEnvironment & env);
    void get_step_actions(const SharedEnvironment & env, vector<Action> & actions);
    // Action get_action_from_states(const State & state, const State & next_state);
    // [end]

    std::shared_ptr<Graph> G; // graph
    std::shared_ptr<HeuristicTable> H; // instance
    Config next_config;

    vector<AgentInfo> agent_infos;

    Executor executor;

    Instance build_instance(const SharedEnvironment & env);
    int get_neighbor_orientation(int loc1,int loc2);

    LaCAMSolver(const std::shared_ptr<HeuristicTable> & H,SharedEnvironment * env,uint random_seed=0):H(H),action_model(env),executor(env),MT(new std::mt19937(random_seed)){};
    ~LaCAMSolver(){
        delete MT;
    };

    Action get_action_from_states(const State & state, const State & next_state){
#ifndef NO_ROT
        assert(state.timestep+1==next_state.timestep);
        
        if (state.location==next_state.location){
            // either turn or wait
            if (state.orientation==next_state.orientation) {
                return Action::W;
            } else if ((state.orientation-next_state.orientation+4)%4==3) {
                return Action::CR;
            } else if ((state.orientation-next_state.orientation+4)%4==1) {
                return Action::CCR;
            } else {
                assert(false);
                return Action::W;
            }
            }
        else {
            return Action::FW;
        }
#else 
        assert(state.timestep+1==next_state.timestep);
        
        for (int i=0;i<5;++i) {
            if (state.location+action_model.moves[i]==next_state.location) {
                return static_cast<Action>(i);
            }
        }

        cerr<<"Cannot get action from invalid movement between state"<<state<<" and "<<next_state<<endl;
        exit(-1);
#endif
    }

};

}