#include "RHCR/interface/RHCRSolver.h"
#include "States.h"
#include "ActionModel.h"
#include <chrono>
#include <thread>
#include "nlohmann/json.hpp"

namespace RHCR {

State convert_state_type(const ::State & state){
    return {state.location,state.timestep,state.orientation};
}

vector<State> convert_states_type(const vector<::State> & states){
    vector<State> new_states;
    for (int i=0;i<states.size();++i) {
        // TODO: rvalue and move constructor?
        new_states.push_back(convert_state_type(states[i]));
    }
    return new_states;
}

Action get_action_from_states(const State & state, const State & next_state, bool consider_rotation){
    assert(state.timestep+1==next_state.timestep);
    // TODO: assert consider_rotation

    if (consider_rotation){
        if (state.location==next_state.location){
            // either turn or wait
            if (state.orientation==next_state.orientation) {
                return Action::W;
            } else if ((state.orientation-next_state.orientation+4)%4==1) {
                return Action::CR;
            } else if ((state.orientation-next_state.orientation+4)%4==3) {
                return Action::CCR;
            } else {
                assert(false);
                return Action::W;
            }
        }
        else {
            return Action::FW;
        }
    } else {
        // TODO
        assert(false);
        return Action::W;
    }
}

void debug_agent(int agent,const vector<State> &starts,const vector< vector<pair<int, int> > > & goal_locations) {
    cerr<<"plan for agent "<<agent<<": ";
    cerr<<"start: "<<starts[agent].location<<". ";
    cerr<<"goals:";
    const vector<pair<int,int> > & goal_location=goal_locations[agent];
    for (int i=0;i<goal_location.size();++i){
        cerr<<" ["<<goal_location[i].second<<"]"<<goal_location[i].first;
    }
    cerr<<"."<<endl;
}

void debug_agent_path(int agent,const vector<Path> & paths,int start_timestep=0) {
    cerr<<"planed path for agent "<<agent<<" since timestep "<<start_timestep<<":";
    const Path & path=paths[agent];
    for (int i=start_timestep;i<paths[agent].size();++i){
        cerr<<" "<<path[i].location<<","<<path[i].orientation;
    }
    cerr<<endl;
}

void RHCRSolver::start_plan_task(){
    // stop_flag=false;
    // plan();
}

void RHCRSolver::stop_plan_task(){
    // if (!stop_flag) {
    //     stop_flag=true;
    //     task.join();
    // }
}

void RHCRSolver::set_parameters(const string & map_name){
    // load configs
	string config_path="configs/"+map_name.substr(0,map_name.find_last_of("."))+".json";
	cerr<<config_path<<endl;

    nlohmann::json data;
    std::ifstream f(config_path);
    try
    {
        data = nlohmann::json::parse(f);
    }
    catch (nlohmann::json::parse_error error)
    {
        std::cerr << "Failed to load " << config_path << std::endl;
        std::cerr << "Message: " << error.what() << std::endl;
        exit(1);
    }

	int planning_window=read_param_json<int>(data,"planning_window");
    this->planning_window=planning_window;
}

void RHCRSolver::initialize(const SharedEnvironment & env){
    // set parameters according to the specific map
    // TODO(hj): move everything about configuration and initialization here
    set_parameters(env.map_name);
    graph.preprocessing(consider_rotation,env.file_storage_path);
    initialize_solvers();
}

void RHCRSolver::update_goal_locations(const SharedEnvironment & env){
    for (int i=0;i<num_of_drives;++i){
        goal_locations[i].clear();
        for (int j=0;j<env.goal_locations[i].size();++j){
            // we require path of at least length one, even the start and the goal are the same.
            goal_locations[i].emplace_back(env.goal_locations[i][j].first,max(env.goal_locations[i][j].second-timestep,1));
        }
    }
}

void RHCRSolver::plan(const SharedEnvironment & env){
    // sleep for test purpose
    // std::this_thread::sleep_for (std::chrono::milliseconds(1000));

    auto _curr_states=convert_states_type(env.curr_states);
    bool need_replan=false;
    if (paths.size()==0) {
        // initialize paths
        starts.resize(num_of_drives);
        goal_locations.resize(num_of_drives);
        paths.resize(num_of_drives);
        for (int i=0;i<num_of_drives;++i) {
            paths[i].push_back(_curr_states[i]);
        }
        need_replan=true;
    } else {
        // for (int i=0;i<num_of_drives;++i) {
        //     assert(paths[i].size()>timestep);
        //     // if the move is not the same as planned (it actually means the last planned move is not valid), then we need to replan
        //     // if (paths[i][timestep]!=_curr_states[i]) {
        //     //     need_replan=true;
        //     //     paths[i][timestep]=_curr_states[i];
        //     // }
        //     // TODO: if goal changes, we need to replan
        // }
        need_replan=true;
    }

    if (need_replan) {
        // clear the previous plan
        for (int i=0;i<num_of_drives;++i) {
            paths[i].resize(timestep+1);
        }           

        update_start_locations();

        // TODO if we want to use hold_endpoints or dummy_paths, we need to adapt codes from KivaSystem::update_goal_locations()!
        update_goal_locations(env);

        // plan
        solve();

        // int agent=7;
        // debug_agent(agent,starts,goal_locations);
        // debug_agent_path(agent,paths,timestep);
 
        cout<<"RHCRSolver solved for timestep "<<timestep<<endl;
    }
}

void RHCRSolver::get_step_actions(const SharedEnvironment & env, vector<Action> & actions){
    // check empty
    assert(actions.empty());

    for (int i=0;i<num_of_drives;++i) {
        // we will get action indexed at timestep+1
        if (paths[i].size()<=timestep+1){
            cerr<<"wierd error for agent "<<i<<". path length: "<<paths[i].size()<<", "<<"timestep+1: "<<timestep+1<<endl;
            assert(false);
        }
        actions.push_back(get_action_from_states(paths[i][timestep],paths[i][timestep+1],consider_rotation));
    }

    // TODO(hj) we probably still want to check the validness. so we need construct model or implement is_valid by ourselves.
    // check if not valid, this should not happen in general if the algorithm is correct? but maybe there exist deadlocks.
    // TODO(hj) detect deadlock?
//     if (!model.is_valid(env.curr_states,actions)){
//         cerr<<"planed actions are not valid in timestep "<<timestep+1<<"!"<<endl;
// #ifdef DEBUG
//         assert(false);
// #else
//         actions.resize(num_of_drives, Action::W);
// #endif
//     } else {
    // NOTE(hj): only successfully executing a planned step will increase this internal timestep, which is different from the real timestep used in the simulation system.
    timestep+=1;
    // }

}
}