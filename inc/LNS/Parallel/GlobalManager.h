#pragma once
#include "LNS/Parallel/NeighborGenerator.h"
#include "LNS/Parallel/LocalOptimizer.h"
#include <memory>

namespace LNS {

class GlobalManager
{
public:
    NeighborGenerator neighbor_generator;
    std::vector<std::shared_ptr<LocalOptimizer>> local_optimizers;

    int initial_sum_of_costs=MAX_COST;
    int sum_of_costs=MAX_COST;
    int num_of_failures=0;
    double average_group_size=0;
    int sum_of_distances = 0;
    int window_size_for_CT;
    int window_size_for_CAT;
    int window_size_for_PATH;
    list<IterationStats> iteration_stats;
    string init_algo_name;
    string replan_algo_name;
    Instance & instance;
    PathTable & path_table;
    std::vector<Agent> & agents;
    std::shared_ptr<HeuristicTable> HT;
    double time_limit;
    int screen;
    int num_threads;

    GlobalManager(
        Instance & instance, PathTable & path_table, std::vector<Agent> & agents, std::shared_ptr<HeuristicTable> HT,
        int neighbor_size, destroy_heuristic destroy_strategy,
        bool ALNS, double decay_factor, double reaction_factor,
        string init_algo_name, string replan_algo_name, bool sipp,
        int window_size_for_CT, int window_size_for_CAT, int window_size_for_PATH,
        double time_limit, int screen
    );

    void getInitialSolution(Neighbor & neighbor);
    bool run();
    void update(Neighbor & neighbor, bool recheck);
    void update(Neighbor & neighbor);

    string getSolverName() const { return "LNS(" + init_algo_name + ";" + replan_algo_name + ")"; }

};

}