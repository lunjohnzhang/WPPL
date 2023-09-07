#pragma once
#include "SIPP.h"
#include "MAPFSolver.h"
#include <ctime>
#include "common.h"
#include <queue>

namespace RHCR {
// WHCA* with random restart
class WHCAStar :
	public MAPFSolver
{
public:

    uint64_t num_expanded;
    uint64_t num_generated;
    uint64_t num_restarts;

    vector<Path> initial_solution;

    // Runs the algorithm until the problem is solved or time is exhausted
    bool run(const vector<State>& starts,
             const vector< vector<pair<int, int> > >& goal_locations,
             int time_limit);

	string get_name() const {return "WHCA"; }

    void save_results(const std::string &fileName, const std::string &instanceName) const;
	void save_search_tree(const std::string &fileName) const {}
	void save_constraints_in_goal_node(const std::string &fileName) const {}
	void clear();

    WHCAStar(const BasicGraph& G, SingleAgentSolver& path_planner);
    ~WHCAStar() {}

    void build_approximate_goals(unordered_map<int,int> &approximate_goals, const pair<int,int> & goal, unordered_set<int> & reservered_locations);

private:
    void print_results() const;
};
} // end namespace RHCR
