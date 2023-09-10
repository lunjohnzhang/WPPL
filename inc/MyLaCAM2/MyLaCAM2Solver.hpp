#pragma once

#include "RHCR/interface/CompetitionActionModel.h"
#include <random>
#include "MyLaCAM2/graph.hpp"
#include "MyLaCAM2/my_instance.hpp"
#include "MyLaCAM2/my_planner.hpp"
#include <memory>
#include "PIBT/HeuristicTable.h"
#include "MyLaCAM2/executor.hpp"

namespace MyLaCAM2 {

class MyLaCAM2Solver {

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
    MyConfig next_config;

    vector<MyAgentInfo> agent_infos;

    Executor executor;

    MyInstance build_instance(const SharedEnvironment & env);
    int get_neighbor_orientation(int loc1,int loc2);

    MyLaCAM2Solver(const std::shared_ptr<HeuristicTable> & HT,SharedEnvironment * env,uint random_seed=0):HT(HT),action_model(env),executor(env),MT(new std::mt19937(random_seed)){};
    ~MyLaCAM2Solver(){
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