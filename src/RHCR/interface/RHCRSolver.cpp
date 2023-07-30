#include "RHCR/interface/RHCRSolver.h"

namespace RHCR {

void RHCRSolver::step(int time_limit, vector<Action> & actions){

}

void RHCRSolver::plan(){

}

void RHCRSolver::get_step_actions(vector<Action> & actions){
    // check
    assert(actions.empty());
    assert(!action_plan.empty());

    // pop the first step of all actions
    const auto & _actions=action_plan.front();
    action_plan.pop_front();
    actions.insert(actions.end(),_actions.begin(),_actions.end());
}

}