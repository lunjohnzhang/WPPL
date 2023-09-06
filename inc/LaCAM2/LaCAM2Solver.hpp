#pragma once

#include "RHCR/interface/CompetitionActionModel.h"
#include <random>
#include "LaCAM2/graph.hpp"
#include "LaCAM2/instance.hpp"
#include "LaCAM2/planner.hpp"
#include <memory>
#include "PIBT/HeuristicTable.h"
#include "LaCAM2/executor.hpp"

namespace LaCAM2 {

class LaCAM2Solver {

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
    std::shared_ptr<HeuristicTable> HT; // instance
    Config next_config;

    vector<AgentInfo> agent_infos;

    Executor executor;

    Instance build_instance(const SharedEnvironment & env);
    int get_neighbor_orientation(int loc1,int loc2);

    LaCAM2Solver(const std::shared_ptr<HeuristicTable> & HT,SharedEnvironment * env,uint random_seed=0):HT(HT),action_model(env),executor(env),MT(new std::mt19937(random_seed)){};
    ~LaCAM2Solver(){
        delete MT;
    };

    Action get_action_from_states(const State & state, const State & next_state){
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
    }

};

}