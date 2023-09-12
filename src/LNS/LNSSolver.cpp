#include "LNS/LNSSolver.h"

namespace LNS {

void LNSSolver::initialize(const SharedEnvironment & env){
    lacam2_solver->initialize(env);
}

void LNSSolver::plan(const SharedEnvironment & env){

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
        pipp_option
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
            cerr<<"agent "<<i<<": ";
            for (int j=0;j<lns.agents[i].path.size();++j){
                cerr<<lacam2_solver->paths[i][j].location<<" ";
            }   
            cerr<<endl;
        }
    }

    bool succ=lns.run();
    if (succ)
    {
        cerr<<"succeed"<<endl;
    }

}

void LNSSolver::get_step_actions(const SharedEnvironment & env, vector<Action> & actions){

}

} // end namespace LNS
