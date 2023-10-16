#pragma once
#include "LNS/LNS.h"
#include <queue>
#include "util/TimeLimiter.h"
#include "util/HeuristicTable.h"

namespace LNS {

namespace Parallel {

class NeighborGenerator
{
public:
    Instance & instance;
    std::shared_ptr<HeuristicTable> HT;
    PathTable & path_table;
    std::vector<Agent> & agents;

    int neighbor_size; // the size of the generated neighbor
    destroy_heuristic destroy_strategy;

    bool ALNS; // whether to use ALNS
    double decay_factor;
    double reaction_factor;

    int screen=0;
    int num_threads;

    static const int n_orients=4; // east, south, west, north

    // for ALNS
    vector<double> destroy_weights; // the weights of each destroy heuristic
    // int selected_neighbor; // TODO: rename? is it just the some kind of selected strategy's id?

    // for all strategies: currently we only use it for parallelism
    // unordered_set<int> global_tabu_list;
    // for randomwalk strategy
    // unordered_set<int> tabu_list;
    std::vector<unordered_set<int>> tabu_list_list; // for randomwalk strategy
    // for intersection strategy: this is read-only after first generation
    list<int> intersections;

    std::vector<std::shared_ptr<Neighbor>> neighbors; // the generated neighbors for usage

    NeighborGenerator(
        Instance & instance, std::shared_ptr<HeuristicTable> HT, PathTable & path_table, std::vector<Agent> & agents, 
        int neighbor_size, destroy_heuristic destroy_strategy, 
        bool ALNS, double decay_factor, double reaction_factor, 
        int num_threads, int screen
    );

    // we will just make this part sequentially now, namely each time we only select one neighborhood
    // if we want to parallelize the optimization of multiple ones. just call this fuction more.
    void generate_parallel(const TimeLimiter & time_limiter);
    void generate(const TimeLimiter & time_limiter,int idx);
    void update(Neighbor & neighbor);

    void chooseDestroyHeuristicbyALNS();
    bool generateNeighborByRandomWalk(Neighbor & neighbor, int idx);
    bool generateNeighborByIntersection(Neighbor & neighbor);

    void reset();

private:
    int rouletteWheel();

    int findMostDelayedAgent(int idx);
    void randomWalk(
        int agent_id, int start_timestep, 
        set<int>& conflicting_agents, int neighbor_size
    );
    std::list<std::pair<int,int> > getSuccessors(int loc, int orient);

};

}

}