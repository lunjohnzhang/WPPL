#include "LNS/LNSSolver.h"

namespace LNS {

void LNSSolver::initialize(const SharedEnvironment & env){

}

void LNSSolver::plan(const SharedEnvironment & env){
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

    bool succ=lns.run();
    if (succ)
    {
        cerr<<"succeed"<<endl;
    }

}

void LNSSolver::get_step_actions(const SharedEnvironment & env, vector<Action> & actions){

}

} // end namespace LNS
