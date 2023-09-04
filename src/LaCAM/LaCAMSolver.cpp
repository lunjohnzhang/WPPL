#include "LaCAM/LaCAMSolver.h"
#include "util/MyLogger.h"
#include "LaCAM/post_processing.h"

namespace LaCAM {

void LaCAMSolver::initialize(const SharedEnvironment & env) {
    paths.resize(env.num_of_agents);
    cerr<<env.map.size()<<endl;
    G = std::make_shared<Graph>(env);
}

Instance LaCAMSolver::build_instance(const SharedEnvironment & env) {
    auto starts=vector<int>();
    auto goals=vector<int>();
    for (int i=0;i<env.num_of_agents;++i) {
        starts.push_back(env.curr_states[i].location);
        assert(env.goal_locations[i].size()>0);
        goals.push_back(env.goal_locations[i][0].first);
        // cerr<<"0\trandom-32-32-20.map\t32\t32\t"<<starts[i]%32<<"\t"<<starts[i]/32<<"\t"<<goals[i]%32<<"\t"<<goals[i]/32<<"\t0"<<endl;
    }
    return Instance(*G, starts, goals);
}

int LaCAMSolver::get_neighbor_orientation(int loc1,int loc2) {

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

void LaCAMSolver::plan(const SharedEnvironment & env){
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
        auto planner = Planner(&instance,H,&deadline,MT,0);
        const auto solution=planner.solve();
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
        print_stats(verbose, instance, H, solution, comp_time_ms);

        cout<<"solution length:"<<solution.size()<<endl;
        
        if (solution.size()==1) {
            next_config=solution[0];
        } else {  
            next_config=solution[1];
        }
    
    }

    bool ready_to_forward = true;
    for (int i=0;i<env.num_of_agents;++i) {
        auto & curr_state = paths[i][timestep];
        if (curr_state.location!=next_config[i]->index) {
            int expected_orient = get_neighbor_orientation(curr_state.location,next_config[i]->index);
            int curr_orient = curr_state.orientation;
            if (expected_orient!=curr_orient){
                ready_to_forward = false;
                break;
            }
        }
        
    }

    cout<<"ready to forward: "<<ready_to_forward<<endl;

    if (!ready_to_forward) {
        for (int i=0;i<env.num_of_agents;++i) {
            auto & curr_state = paths[i][timestep];
            if (curr_state.location==next_config[i]->index) {
                paths[i].emplace_back(curr_state.location,curr_state.timestep+1,curr_state.orientation);
            } else {
                int expected_orient = get_neighbor_orientation(curr_state.location,next_config[i]->index);
                int curr_orient = curr_state.orientation;
                if (expected_orient==curr_orient){
                    paths[i].emplace_back(curr_state.location,curr_state.timestep+1,expected_orient);
                } else {
                    int d1=(curr_orient+4-expected_orient)%4;
                    int d2=(expected_orient+4-curr_orient)%4;

                    int next_orient=-1;
                    if (d1<d2) {
                        next_orient=(curr_orient-1+4)%4;
                    } else {
                        next_orient=(curr_orient+1+4)%4;
                    }


                    if (i==14) {
                        cerr<<d1<<" "<<d2<<" "<<curr_orient<<" "<<next_orient<<endl;
                    }

                    paths[i].emplace_back(curr_state.location,curr_state.timestep+1,next_orient);
                }
            }
        }
    } else {
        for (int i=0;i<env.num_of_agents;++i) {
            auto & curr_state = paths[i][timestep];
            if (curr_state.location==next_config[i]->index) {
                paths[i].emplace_back(curr_state.location,curr_state.timestep+1,curr_state.orientation);
            } else {
                paths[i].emplace_back(next_config[i]->index,curr_state.timestep+1,curr_state.orientation);
            }
        }
    }

    // total_feasible_timestep+=1;

}


void LaCAMSolver::get_step_actions(const SharedEnvironment & env, vector<Action> & actions) {
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

    for (int i=0;i<env.num_of_agents;++i) {
        cout<<actions[i]<<" ";
    }
    cout<<endl;

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
    
    // need_replan=true;
    
}



}