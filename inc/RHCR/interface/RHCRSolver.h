#pragma once
#include "common.h"
#include "ActionModel.h"
#include "RHCR/main/MAPFSolver.h"
#include "RHCR/main/BasicGraph.h"

namespace RHCR {
class RHCRSolver{
// for simplicity, just make everything public. but attributes or functions start with _ are supposed to use privately in general cases.
public:

    //***** attributes *****//

    shared_ptr<MAPFSolver> mapf_solver;
    shared_ptr<BasicGraph> graph;

    // this deque maintains actions planned into future. we can directly retrieve the actions for the current step from this deque.
    deque<vector<Action>> action_plan;


    //***** functions *****//
    RHCRSolver(const shared_ptr<MAPFSolver> & mapf_solver, const shared_ptr<BasicGraph> & graph):mapf_solver(mapf_solver),graph(graph){};

    // this function should be called every step. it will call plan() function when necessary and retrieve actions for the current step.
    void step(int time_limit, vector<Action> & actions);

    // this function plans actions into future (possible from a certain state?)
    void plan();

    void get_step_actions(vector<Action> & actions);

    // inline void set_mapf_solver(const shared_ptr<MAPFSolver> & mapf_solver){
    //     this->mapf_solver=mapf_solver;
    // };

    // inline void set_map(const shared_ptr<BasicGraph> & map){
    //     this->map=map;
    // };
};
}