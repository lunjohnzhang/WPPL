#include "RHCR/interface/RHCRSolver.h"
#include "States.h"
#include "ActionModel.h"

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

Action get_action_from_states(const State & state, const State & next_state){
    assert(state.timestep+1==next_state.timestep);
    // TODO: assert consider_rotation

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

void RHCRSolver::initialize(){
    graph.preprocessing(consider_rotation);
    initialize_solvers();
}

void RHCRSolver::update_goal_locations(const SharedEnvironment & env){
    for (int i=0;i<num_of_drives;++i){
        goal_locations[i].clear();
        for (int j=0;j<env.goal_locations[i].size();++j){
            goal_locations[i].emplace_back(env.goal_locations[i][j].first,env.goal_locations[i][j].second-timestep);
        }
    }
}

void RHCRSolver::plan(const SharedEnvironment & env){
    timestep=env.curr_timestep;

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
        // TODO check whether the move is the same as planned. if not, we need to clean the plan and then replan somehow.
        // for (int i=0;i<num_of_drives;++i) {
        //     if (paths[i][timestep]!=_curr_states[i]) {
        //         cerr<<"not same!!!"<<endl;
        //         need_replan=true;
        //         paths[i][timestep]=_curr_states[i];
        //     }
        // }

        // TODO remove it. currently we replan for every timestep.
        need_replan=true;

        if (need_replan) {
            // clear the previous plan
            for (int i=0;i<num_of_drives;++i) {
                paths[i].resize(timestep+1);
            }           
        }
    }

    update_start_locations();

    // TODO if we want to use hold_endpoints or dummy_paths, we need to copy codes from KivaSystem::update_goal_locations()!
    // the timestep is not correct in my understanding
    // goal_locations=env.goal_locations;
    update_goal_locations(env);

    // plan
    solve();

    // for (int i=0;i<paths[2].size();++i){
    //     cerr<<paths[2][i].location<<","<<paths[2][i].orientation<<" ";
    // }
    // cerr<<endl;

    cout<<"RHCRSolver solved for timestep "<<timestep<<endl;
}

void RHCRSolver::get_step_actions(const SharedEnvironment & env, vector<Action> & actions){
    // check empty
    assert(actions.empty());

    for (int i=0;i<num_of_drives;++i) {
        // we will get action indexed at timestep+1
        assert(paths[i].size()>timestep+1);
        actions.push_back(get_action_from_states(paths[i][timestep],paths[i][timestep+1]));
    }

    // check if not valid, this should not happen in general if the algorithm is correct? but maybe there exist deadlocks.
    // TODO detect deadlock?
    if (!model.is_valid(env.curr_states,actions)){
        cerr<<"planed actions are not valid in timestep "<<timestep+1<<"!"<<endl;
#ifdef DEBUG
        assert(false);
#else
        actions.resize(num_of_drives, Action::W);
#endif
    }

}
}