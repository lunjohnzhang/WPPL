#include "LaCAM2/LaCAM2Solver.hpp"
#include "util/MyLogger.h"
#include "LaCAM2/post_processing.hpp"
#include "PIBT/util.h"

namespace LaCAM2 {

void LaCAM2Solver::initialize(const SharedEnvironment & env) {
    paths.resize(env.num_of_agents);
    agent_infos.resize(env.num_of_agents);
    G = std::make_shared<Graph>(env);
}

Instance LaCAM2Solver::build_instance(const SharedEnvironment & env) {
    auto starts=vector<uint>();
    auto goals=vector<uint>();
    for (int i=0;i<env.num_of_agents;++i) {
        starts.push_back(env.curr_states[i].location);
        assert(env.goal_locations[i].size()>0);
        int goal_location=env.goal_locations[i][0].first;
        goals.push_back(goal_location);
        auto & agent_info=agent_infos[i];
        // cerr<<"0\trandom-32-32-20.map\t32\t32\t"<<starts[i]%32<<"\t"<<starts[i]/32<<"\t"<<goals[i]%32<<"\t"<<goals[i]/32<<"\t0"<<endl;
        if (goal_location!=agent_info.goal_location){
            agent_info.goal_location=goal_location;
            agent_info.elapsed=0;
            agent_info.tie_breaker=getRandomFloat(0,1,MT);
        } else {
            agent_info.elapsed+=1;
        }
    }
    return Instance(*G, starts, goals, agent_infos, read_param_json<int>(config,"planning_window",-1));
}

int LaCAM2Solver::get_neighbor_orientation(int loc1,int loc2) {

    // 0:east, 1:south, 2:west, 3:north

    if (loc1+1==loc2) {
        return 0;
    }

    if (loc1+G->width==loc2) {
        return 1;
    }

    if (loc1-1==loc2) {
        return 2;
    }

    if (loc1-G->width==loc2) {
        return 3;
    }

    cerr<<"loc1 and loc2 are not neighbors: "<<loc1<<", "<<loc2<<endl;
    exit(-1);

}

void LaCAM2Solver::plan(const SharedEnvironment & env){
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
        bool use_swap=false;
        bool use_orient_in_heuristic=true;
        auto planner = Planner(&instance,HT,&deadline,MT,0,LaCAM2::OBJ_SUM_OF_LOSS,0.001F,use_swap,use_orient_in_heuristic);
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
        print_stats(verbose, instance, HT, solution, comp_time_ms);

        cout<<"solution length:"<<solution.size()<<endl;
        
        if (solution.size()==1) {
            next_config=solution[0];
        } else {  
            next_config=solution[1];
        }
    

        if (!read_param_json<bool>(config,"consider_rotation")) {
            for (int i=0;i<env.num_of_agents;++i) {
                cerr<<"xagent "<<i<<": ";
                for (int j=1;j<solution.size();++j) {
                    cerr<<solution[j][i]->index<<" ";
                }
                cerr<<endl;
                for (int j=1;j<solution.size();++j) {
                    paths[i].emplace_back(solution[j][i]->index,env.curr_states[i].timestep+1+j,-1);
                }
            }
        }
    }

    if (read_param_json<bool>(config,"consider_rotation")) {
        vector<State> planned_next_states;
        vector<State> next_states;
        for (int i=0;i<env.num_of_agents;++i) {
            planned_next_states.emplace_back(next_config[i]->index,-1,-1);
            next_states.emplace_back(-1,-1,-1);
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

    // bool ready_to_forward = true;
    // for (int i=0;i<env.num_of_agents;++i) {
    //     auto & curr_state = paths[i][timestep];
    //     if (curr_state.location!=next_config[i]->index) {
    //         int expected_orient = get_neighbor_orientation(curr_state.location,next_config[i]->index);
    //         int curr_orient = curr_state.orientation;
    //         if (expected_orient!=curr_orient){
    //             ready_to_forward = false;
    //             break;
    //         }
    //     }
        
    // }

    // cout<<"ready to forward: "<<ready_to_forward<<endl;

    // if (!ready_to_forward) {
    //     for (int i=0;i<env.num_of_agents;++i) {
    //         auto & curr_state = paths[i][timestep];
    //         if (curr_state.location==next_config[i]->index) {
    //             paths[i].emplace_back(curr_state.location,curr_state.timestep+1,curr_state.orientation);
    //         } else {
    //             int expected_orient = get_neighbor_orientation(curr_state.location,next_config[i]->index);
    //             int curr_orient = curr_state.orientation;
    //             if (expected_orient==curr_orient){
    //                 paths[i].emplace_back(curr_state.location,curr_state.timestep+1,expected_orient);
    //             } else {
    //                 int d1=(curr_orient+4-expected_orient)%4;
    //                 int d2=(expected_orient+4-curr_orient)%4;

    //                 int next_orient=-1;
    //                 if (d1<d2) {
    //                     next_orient=(curr_orient-1+4)%4;
    //                 } else {
    //                     next_orient=(curr_orient+1+4)%4;
    //                 }
    //                 paths[i].emplace_back(curr_state.location,curr_state.timestep+1,next_orient);
    //             }
    //         }
    //     }
    // } else {
    //     for (int i=0;i<env.num_of_agents;++i) {
    //         auto & curr_state = paths[i][timestep];
    //         if (curr_state.location==next_config[i]->index) {
    //             paths[i].emplace_back(curr_state.location,curr_state.timestep+1,curr_state.orientation);
    //         } else {
    //             paths[i].emplace_back(next_config[i]->index,curr_state.timestep+1,curr_state.orientation);
    //         }
    //     }
    // }

    // total_feasible_timestep+=1;

}


void LaCAM2Solver::get_step_actions(const SharedEnvironment & env, vector<Action> & actions) {
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

    // for (int i=0;i<env.num_of_agents;++i) {
    //     cout<<actions[i]<<" ";
    // }
    // cout<<endl;

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

    bool all_arrived=true;
    for (int i=0;i<env.num_of_agents;++i) {
        if (paths[i][timestep].location!=next_config[i]->index) {
            // arrive goal locations
            all_arrived=false;
            break;
        }
    }    
    if (all_arrived) {
        need_replan=true;
    }

    // 1. exceed simulation window
    // if (timestep==total_feasible_timestep){
    //     need_replan=true;
    // }
    
    // 2. goal changes: there different ways to check this. let's just keep the old goal and compare.
    for (int i=0;i<env.num_of_agents;++i) {
        if (paths[i][timestep].location==env.goal_locations[i][0].first) {
            // arrive goal locations
            need_replan=true;
            break;
        }
    }
    
    need_replan=true;
    
}



}