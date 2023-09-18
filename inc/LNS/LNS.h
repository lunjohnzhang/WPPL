#pragma once
#include "LNS/BasicLNS.h"
#include "LNS/InitLNS.h"

//pibt related
#include "LNS/PIBT/simplegrid.h"
#include "LNS/PIBT/pibt_agent.h"
#include "LNS/PIBT/problem.h"
#include "LNS/PIBT/mapf.h"
#include "LNS/PIBT/pibt.h"
#include "LNS/PIBT/pps.h"
#include "LNS/PIBT/winpibt.h"
#include "PIBT/HeuristicTable.h"
#include <memory>

namespace LNS {

enum destroy_heuristic { RANDOMAGENTS, RANDOMWALK, INTERSECTION, DESTORY_COUNT };

// TODO: adaptively change the neighbor size, that is,
// increase it if no progress is made for a while
// decrease it if replanning fails to find any solutions for several times

class LNS : public BasicLNS
{
public:
    vector<Agent> agents;
    double preprocessing_time = 0;
    double initial_solution_runtime = 0;
    int initial_sum_of_costs = -1;
    int sum_of_costs_lowerbound = -1;
    int sum_of_distances = -1;
    int restart_times = 0;

    int window_size_for_CT = MAX_TIMESTEP;
    int window_size_for_CAT = MAX_TIMESTEP;
    int window_size_for_PATH = MAX_TIMESTEP;
    
    std::shared_ptr<HeuristicTable> HT; // instance

    LNS(const Instance& instance, double time_limit,
        const string & init_algo_name, const string & replan_algo_name, const string & destory_name,
        int neighbor_size, int num_of_iterations, bool init_lns, const string & init_destory_name, bool use_sipp,
        int screen, PIBTPPS_option pipp_option, const std::shared_ptr<HeuristicTable> & HT, 
        int window_size_for_CT = MAX_TIMESTEP, int window_size_for_CAT = MAX_TIMESTEP, int window_size_for_PATH = MAX_TIMESTEP);
    ~LNS()
    {
        delete init_lns;
    }
    bool getInitialSolution();
    bool run();
    void validateSolution() const;
    void writeIterStatsToFile(const string & file_name) const;
    void writeResultToFile(const string & file_name) const;
    void writePathsToFile(const string & file_name) const;
    string getSolverName() const override { return "LNS(" + init_algo_name + ";" + replan_algo_name + ")"; }
private:
    InitLNS* init_lns = nullptr;
    string init_algo_name;
    string replan_algo_name;
    bool use_init_lns; // use LNS to find initial solutions
    destroy_heuristic destroy_strategy = RANDOMWALK;
    int num_of_iterations;
    string init_destory_name;
    PIBTPPS_option pipp_option;


    PathTableWC path_table_wc;
    PathTable path_table; // 1. stores the paths of all agents in a time-space table;
    // 2. avoid making copies of this variable as much as possible.
    unordered_set<int> tabu_list; // used by randomwalk strategy
    list<int> intersections;

    bool runEECBS();
    bool runCBS();
    bool runPP(bool init_run = false);
    bool runPIBT();
    bool runPPS();
    bool runWinPIBT();
    bool checkPrecomputed();

    MAPF preparePIBTProblem(vector<int>& shuffled_agents);
    void updatePIBTResult(const PIBT_Agents& A, vector<int>& shuffled_agents);

    void chooseDestroyHeuristicbyALNS();

    bool generateNeighborByRandomWalk();
    bool generateNeighborByIntersection();

    int findMostDelayedAgent();
    int findRandomAgent() const;
    void randomWalk(int agent_id, int start_location, int start_timestep,
                    set<int>& neighbor, int neighbor_size, int upperbound);
};

} // end namespace LNS