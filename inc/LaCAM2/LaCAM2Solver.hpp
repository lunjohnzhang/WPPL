#pragma once

#include "util/CompetitionActionModel.h"
#include <random>
#include "LaCAM2/graph.hpp"
#include "LaCAM2/instance.hpp"
#include "LaCAM2/planner.hpp"
#include <memory>
#include "util/HeuristicTable.h"
#include "LaCAM2/executor.hpp"
#include "LaCAM2/slow_executor.hpp"
#include "nlohmann/json.hpp"

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
    void plan(const SharedEnvironment & env, std::vector<Path> * precomputed_paths=nullptr);
    void get_step_actions(const SharedEnvironment & env, vector<Action> & actions);
    // Action get_action_from_states(const State & state, const State & next_state);
    // [end]

    std::shared_ptr<Graph> G; // graph
    std::shared_ptr<HeuristicTable> HT; // instance
    Config next_config;

    vector<AgentInfo> agent_infos;

    Executor executor;
    SlowExecutor slow_executor;

    nlohmann::json config;

    Instance build_instance(const SharedEnvironment & env, std::vector<Path> * precomputed_paths=nullptr);
    int get_neighbor_orientation(int loc1,int loc2);

    LaCAM2Solver(const std::shared_ptr<HeuristicTable> & HT, SharedEnvironment * env, nlohmann::json & config):
        HT(HT),action_model(env),executor(env),slow_executor(env),
        config(config),
        MT(new std::mt19937(read_param_json<uint>(config,"seed",0))){

    };

    ~LaCAM2Solver(){
        delete MT;
    };


    void clear() {
        int num_of_agents=paths.size();

        paths.clear();
        need_replan = true;
        total_feasible_timestep = 0;
        timestep = 0;
        delete MT;
        MT = new std::mt19937(read_param_json(config,"seed",0));
        next_config = Config();
        agent_infos.clear();

        paths.resize(num_of_agents);
        agent_infos.resize(num_of_agents);
    }

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

        cerr<<"Cannot get action from invalid movement between state "<<state<<" and "<<next_state<<endl;
        exit(-1);
#endif
    }

};

}