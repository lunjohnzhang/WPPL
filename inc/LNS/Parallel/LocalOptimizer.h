#pragma once
#include "LNS/PathTable.h"
#include "LNS/BasicLNS.h"
#include "util/HeuristicTable.h"

namespace LNS {

class LocalOptimizer
{
public:
    // TODO(rivers): think about what data structure needs a separate copy for each local optimizer.
    Instance & instance;
    PathTable path_table; // maintain a copy
    std::vector<Agent> & agents; // remove in the future, currently we can visit it for agent id but not do anything else.
    std::shared_ptr<HeuristicTable> HT;
    std::shared_ptr<SingleAgentSolver> path_planner; // maintain

    string replan_algo_name;
    int window_size_for_CT;
    int window_size_for_CAT;
    int window_size_for_PATH;

    int screen=0;

    LocalOptimizer(
        Instance & instance, std::vector<Agent> & agents, std::shared_ptr<HeuristicTable> HT,
        string replan_algo_name, bool sipp,
        int window_size_for_CT, int window_size_for_CAT, int window_size_for_PATH,
        int screen
    );

    // the global manager will push global changes to local optimizer though this function. 
    void update(Neighbor & neighbor);
    void optimize(Neighbor & neighbor, double time_limit);
    void prepare(Neighbor & neighbor);

    bool runPP(Neighbor & neighbor, double time_limit);

};

}