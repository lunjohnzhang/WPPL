#include "LNS/LNSSolver.h"

namespace LNS {

void LNSSolver::initialize(const SharedEnvironment & env){
    lacam2_solver->initialize(env);
    paths.resize(env.num_of_agents);
    executed_plan_step = -1;
}

void LNSSolver::plan(const SharedEnvironment & env){
    // should be moved outside.
    observe(env);

    if (need_replan) {

        if (read_param_json<string>(config,"initAlgo")=="LaCAM2"){
            // use lacam2 to get a initial solution
            // TODO: it is possible easier we clear everything first of lacam2
            lacam2_solver->clear();
            lacam2_solver->plan(env);
        }

        // build instace
        Instance instance(env);
        modify_goals(instance.goal_locations, env);

        // build planner
        PIBTPPS_option pipp_option;
        pipp_option.windowSize = read_param_json<int>(config,"pibtWindow");
        pipp_option.winPIBTSoft = read_param_json<int>(config,"winPibtSoftmode");

        LNS lns(
            instance,
            read_param_json<double>(config,"cutoffTime"),
            read_param_json<string>(config,"initAlgo"),
            read_param_json<string>(config,"replanAlgo"),
            read_param_json<string>(config,"destoryStrategy"),
            read_param_json<int>(config,"neighborSize"),
            read_param_json<int>(config,"maxIterations"),
            read_param_json<bool>(config,"initLNS"),
            read_param_json<string>(config,"initDestoryStrategy"),
            read_param_json<bool>(config,"sipp"),
            read_param_json<int>(config,"screen"),
            pipp_option,
            HT
        );


        if (read_param_json<string>(config,"initAlgo")=="LaCAM2"){
            // copy results into LNS.agents
            for (int i=0;i<lns.agents.size();i++){
                if (lns.agents[i].id!=i) {
                    cerr<<"agents are not ordered at the begining"<<endl;
                    exit(-1);
                }
                lns.agents[i].path.clear();
                bool goal_arrived=false;
                for (int j=0;j<lacam2_solver->paths[i].size();++j){
                    lns.agents[i].path.emplace_back(lacam2_solver->paths[i][j].location);
                    if (lacam2_solver->paths[i][j].location==env.goal_locations[i][0].first){
                        goal_arrived=true;
                        // break;
                    }
                }
                // cerr<<"agent "<<i<<": ";
                // for (int j=0;j<lns.agents[i].path.size();++j){
                //     cerr<<lacam2_solver->paths[i][j].location<<" ";
                // }   
                // cerr<<endl;
            }
        }

        bool succ=lns.run();
        if (succ)
        {
            cout<<"succeed"<<endl;
        }

        // just save to paths: currently we replan at every time step, so we only need to save the first step
        for (int i=0;i<paths.size();++i){
            paths[i].emplace_back(lns.agents[i].path[1].location,-1,-1);
        }

        // for (int i=0;i<paths.size();++i){
        //     cerr<<executed_plan_step<<" "<<env.curr_states[i].location<<" "<<env.curr_states[i].orientation<<endl;
        //     cerr<<"agent "<<i<<": ";
        //     for (int j=0;j<paths[i].size();++j){
        //         cerr<<paths[i][j].location<<" ";
        //     }   
        //     cerr<<endl;
        // }


    }

}

void LNSSolver::observe(const SharedEnvironment & env){
    for (int i=0;i<env.num_of_agents;++i) {
        paths[i].clear();
    }    

    if (paths[0].size()==0) {
        for (int i=0;i<env.num_of_agents;++i) {
            paths[i].push_back(env.curr_states[i]);
        }
    }

    need_replan=true;
    executed_plan_step=0;

    // need_replan=false;

    // // TODO: we need to check the state is at leat the previous one in the plan or the current one.
    // // otherwise, it goes out of control.
    // // update current executed_plan_step if current state match with the plan
    // bool match=true;
    // for (int i=0;i<paths.size();++i){
    //     if (executed_plan_step+1>=paths[i].size()){
    //         // todo
    //         cerr<<"executed_plan_step exceed the plan:"<<executed_plan_step+1<<paths[i].size()<<endl;
    //         exit(-1);
    //     }
    //     if (paths[i][executed_plan_step+1].location!=env.curr_states[i].location){
    //         match=false;
    //     }
    // }
    // if (match){
    //     ++executed_plan_step;
    //     need_replan=true;
    // }
}

void LNSSolver::get_step_actions(const SharedEnvironment & env, vector<Action> & actions){
    assert(actions.empty());

    // get current state and current timestep
    vector<State> planned_next_states;
    vector<State> next_states;
    int next_plan_step=executed_plan_step+1;
    for (int i=0;i<env.num_of_agents;++i) {
        if (next_plan_step>=paths[i].size()){
            // todo: we need to wait for the new plan to come out.
            exit(-1);
        }    
        planned_next_states.emplace_back(paths[i][next_plan_step].location,-1,-1);
        next_states.emplace_back(-1,-1,-1);
    }

    executor.execute(&(env.curr_states),&planned_next_states,&next_states);

    for (int i=0;i<env.num_of_agents;++i) {
        if (next_states[i].timestep!=env.curr_states[i].timestep+1) {
            std::cerr<<i<<" "<<next_states[i].timestep<<" "<<env.curr_states[i].timestep<<endl;
            exit(-1);
        }
    }

    // get actions from current state and next state
    for (int i=0;i<env.num_of_agents;++i) {
        // we will get action indexed at timestep+1
        if (paths[i].size()<=timestep+1){
            cerr<<"wierd error for agent "<<i<<". path length: "<<paths[i].size()<<", "<<"timestep+1: "<<timestep+1<<endl;
            assert(false);
        }
        actions.push_back(get_action_from_states(env.curr_states[i],next_states[i]));
    }

}

} // end namespace LNS
