#pragma once
#include "LNS/common.h"
#include "LNS/SpaceTimeAStar.h"
#include "LNS/SIPP.h"
#include "PIBT/HeuristicTable.h"
#include <memory>

namespace LNS {

struct Agent
{
    int id;
    SingleAgentSolver* path_planner = nullptr; // start, goal, and heuristics are stored in the path planner
    Path path;
    std::shared_ptr<HeuristicTable> HT; // instance

    Agent(const Instance& instance, int id, bool sipp, const std::shared_ptr<HeuristicTable> & HT): id(id), HT(HT)
    {
        if(sipp)
            path_planner = new SIPP(instance, id, HT);
        else
            path_planner = new SpaceTimeAStar(instance, id, HT);
    }
    ~Agent(){ delete path_planner; }

    int getNumOfDelays() const
    {
        return (int) path.size() - 1 - path_planner->my_heuristic[path_planner->start_location];
    }
};

struct Neighbor
{
    vector<int> agents;
    int sum_of_costs;
    int old_sum_of_costs;
    set<pair<int, int>> colliding_pairs;  // id1 < id2
    set<pair<int, int>> old_colliding_pairs;  // id1 < id2
    vector<Path> old_paths;
};

class BasicLNS
{
public:
    // statistics
    int num_of_failures = 0; // #replanning that fails to find any solutions
    list<IterationStats> iteration_stats; //stats about each iteration
    double runtime = 0;
    double average_group_size = -1;
    int sum_of_costs = 0;

    BasicLNS(const Instance& instance, double time_limit, int neighbor_size, int screen);
    virtual string getSolverName() const = 0;
protected:
    // input params
    const Instance& instance; // avoid making copies of this variable as much as possible
    double time_limit;
    double replan_time_limit; // time limit for replanning
    int neighbor_size;
    int screen;

    // adaptive LNS
    bool ALNS = false;
    double decay_factor = -1;
    double reaction_factor = -1;
    vector<double> destroy_weights;
    int selected_neighbor;

    // helper variables
    high_resolution_clock::time_point start_time;
    Neighbor neighbor;

    void rouletteWheel();
};
} // end namespace LNS