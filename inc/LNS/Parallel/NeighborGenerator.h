#pragma once
#include "LNS/LNS.h"
#include <queue>

namespace LNS {

class NeighborGenerator
{
public:
    Instance & instance;
    PathTable & path_table;
    std::vector<Agent> & agents;

    int neighbor_size; // the size of the generated neighbor
    destroy_heuristic destroy_strategy;

    bool ALNS; // whether to use ALNS
    double decay_factor;
    double reaction_factor;

    int screen=0;

    // for ALNS
    vector<double> destroy_weights; // the weights of each destroy heuristic
    int selected_neighbor; // TODO: rename? is it just the some kind of selected strategy's id?

    // for all strategies: currently we only use it for parallelism
    // unordered_set<int> global_tabu_list;
    // for randomwalk strategy
    unordered_set<int> tabu_list;
    // for intersection strategy
    list<int> intersections;

    std::vector<std::shared_ptr<Neighbor>> neighbors; // the generated neighbors for usage

    NeighborGenerator(
        Instance & instance, PathTable & path_table, std::vector<Agent> & agents, 
        int neighbor_size, destroy_heuristic destroy_strategy, 
        bool ALNS, double decay_factor, double reaction_factor, 
        int screen
    );

    // we will just make this part sequentially now, namely each time we only select one neighborhood
    // if we want to parallelize the optimization of multiple ones. just call this fuction more.
    void generate(int n, double time_limit);
    void generate(double time_limit);
    void update(Neighbor & neighbor);

    void chooseDestroyHeuristicbyALNS();
    bool generateNeighborByRandomWalk(Neighbor & neighbor);
    bool generateNeighborByIntersection(Neighbor & neighbor);

private:
    void rouletteWheel();

    int findMostDelayedAgent();
    void randomWalk(
        int agent_id, int start_location, int start_timestep, 
        set<int>& conflicting_agents, int neighbor_size, int upperbound
    );

};

}