#include "MyLaCAM2/MyLaCAM2Solver.hpp"
#include "util/MyLogger.h"
#include "MyLaCAM2/post_processing.hpp"
#include "PIBT/util.h"

namespace MyLaCAM2 {

void MyLaCAM2Solver::initialize(const SharedEnvironment & env) {
    paths.resize(env.num_of_agents);
    agent_infos.resize(env.num_of_agents);
    G = std::make_shared<Graph>(env);
}

MyInstance MyLaCAM2Solver::build_instance(const SharedEnvironment & env) {
    for (int i=0;i<env.num_of_agents;++i) {
        assert(env.goal_locations[i].size()>0);
        int goal_location=env.goal_locations[i][0].first;
        auto & agent_info=agent_infos[i];
        // cerr<<"0\trandom-32-32-20.map\t32\t32\t"<<starts[i]%32<<"\t"<<starts[i]/32<<"\t"<<goals[i]%32<<"\t"<<goals[i]/32<<"\t0"<<endl;
        agent_info.start_state=MyState(G->U[env.curr_states[i].location],-1,-1);
        if (goal_location!=agent_info.goal_location){
            agent_info.goal_location=goal_location;
            agent_info.elapsed=0;
            agent_info.tie_breaker=getRandomFloat(0,1,MT);
        } else {
            agent_info.elapsed+=1;
        }
    }
    return MyInstance(*G, agent_infos);
}

void MyLaCAM2Solver::plan(const SharedEnvironment & env){
    if (timestep==0) {
        for (int i=0;i<env.num_of_agents;++i) {
            paths[i].push_back(env.curr_states[i]);
        }
    }

    if (need_replan) {
        const int verbose = 10;
        const int time_limit_sec = 2;
        auto instance = build_instance(env);
        const auto deadline = Deadline(time_limit_sec * 1000);
        bool use_orient_in_heuristic=true;
        bool use_dist_in_priority=true;
        auto planner = MyPlanner(&instance,HT,&deadline,MT,0,MyLaCAM2::OBJ_SUM_OF_LOSS,0.001F,use_orient_in_heuristic,use_dist_in_priority);
        auto additional_info = std::string("");
        const auto solution=planner.solve(additional_info);
        const auto comp_time_ms = deadline.elapsed_ms();

        // // failure
        // if (solution.empty()) {
        //     info(1, verbose, "failed to solve");
        //     exit(1);
        // }

        // // check feasibility
        // if (!is_feasible_solution(instance, solution, verbose)) {
        //     info(0, verbose, "invalid solution");
        //     exit(2);
        // }

        // post processing
        // print_stats(verbose, instance, HT, solution, comp_time_ms);

        cout<<"solution length:"<<solution.size()<<endl;
        
        if (solution.size()==1) {
            next_config=solution[0];
        } else {  
            next_config=solution[1];
        }
    }

    vector<State> planned_next_states;
    vector<State> next_states;
    for (int i=0;i<env.num_of_agents;++i) {
        planned_next_states.emplace_back(next_config[i]->v->index,-1,-1);
        next_states.emplace_back(-1,-1,-1);
        // std::cerr<<i<<" "<<env.curr_states[i].location<<" "<<next_config[i]->v->index<<endl;
    }

    executor.execute(&(env.curr_states),&planned_next_states,&next_states);

    for (int i=0;i<env.num_of_agents;++i) {
        if (next_states[i].timestep!=env.curr_states[i].timestep+1) {
            std::cerr<<i<<" "<<next_states[i].timestep<<" "<<env.curr_states[i].timestep<<endl;
            exit(-1);
        }

        paths[i].emplace_back(next_states[i]);
        // std::cerr<<i<<" "<<env.curr_states[i]<<" "<<next_states[i]<<endl;
    }

}


void MyLaCAM2Solver::get_step_actions(const SharedEnvironment & env, vector<Action> & actions) {
    // check empty
    assert(actions.empty());

    for (int i=0;i<env.num_of_agents;++i) {
        // we will get action indexed at timestep+1
        if (paths[i].size()<=timestep+1){
            cerr<<"wierd error for agent "<<i<<". path length: "<<paths[i].size()<<", "<<"timestep+1: "<<timestep+1<<endl;
            assert(false);
        }
        actions.push_back(get_action_from_states(paths[i][timestep],paths[i][timestep+1]));
    }

    // TODO(hj) we probably still want to check the validness. so we need construct model or implement is_valid by ourselves.
    // check if not valid, this should not happen in general if the algorithm is correct? but maybe there exist deadlocks.
    // TODO(hj) detect deadlock?
    if (!action_model.is_valid(env.curr_states,actions)){
        cerr<<"planed actions are not valid in timestep "<<timestep+1<<"!"<<endl;
#ifdef DEV
        exit(-1);
#else
        actions.resize(env.num_of_agents, Action::W);
#endif
    } else {
        // NOTE(hj): only successfully executing a planned step will increase this internal timestep, which is different from the real timestep used in the simulation system.
        timestep+=1;
    }

    // TODO(hj): when we need to replan?
    need_replan=false;

    // bool all_arrived=true;
    // for (int i=0;i<env.num_of_agents;++i) {
    //     if (paths[i][timestep].location!=next_config[i]->v->index) {
    //         // arrive goal locations
    //         all_arrived=false;
    //         break;
    //     }
    // }    
    // if (all_arrived) {
    //     need_replan=true;
    // }

    // 1. exceed simulation window
    // if (timestep==total_feasible_timestep){
    //     need_replan=true;
    // }
    
    // 2. goal changes: there different ways to check this. let's just keep the old goal and compare.
    // for (int i=0;i<env.num_of_agents;++i) {
    //     if (paths[i][timestep].location==env.goal_locations[i][0].first) {
    //         // arrive goal locations
    //         need_replan=true;
    //         break;
    //     }
    // }
    
    need_replan=true;
    
}



}