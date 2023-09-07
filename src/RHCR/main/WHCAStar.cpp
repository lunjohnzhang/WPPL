#include "RHCR/main/WHCAStar.h"

namespace RHCR {

WHCAStar::WHCAStar(const BasicGraph &G, SingleAgentSolver& path_planner) : MAPFSolver(G, path_planner) {}




vector<int> get_neighbors(const BasicGraph& G, int curr) {
    vector<int> neighbors;
    int x=curr%G.cols;
    int y=curr/G.cols;

    if (x<G.cols-1) {
        neighbors.push_back(curr+1);
    }
    if (y<G.rows-1) {
        neighbors.push_back(curr+G.cols);
    }
    if (x>0) {
        neighbors.push_back(curr-1);
    }
    if (y>0) {
        neighbors.push_back(curr-G.cols);
    }
    // cerr<<"neighbors:"<<neighbors.size()<<endl;
    return neighbors;

}

void WHCAStar::build_approximate_goals(unordered_map<int,int> & approximate_goals, const pair<int,int> & goal, unordered_set<int> & reservered_locations) {
    // we need a priortized queue to do a backward search.
    std::vector<bool> visited(G.size(),false);;
    std::vector<int> dists(G.size(),INT_MAX);;
    std::queue<int> Q;

    int start=goal.first;
    Q.push(start);
    visited[start]=1;
    dists[start]=0;

    bool found=false;
    int found_dist=-1;

    int cnt=0;
    while (!Q.empty()) {
        int curr=Q.front();
        Q.pop();
        ++cnt;
        // cerr<<cnt<<" "<<curr<<endl;

        // check availability
        if (!found){
            if (reservered_locations.find(curr)==reservered_locations.end()) {
                found=true;
                found_dist=dists[curr];
                approximate_goals.insert({curr,found_dist});
            }
        } else {
            if (dists[curr]==found_dist) {
                approximate_goals.insert({curr,found_dist});
            } else if (dists[curr]>found_dist) {
                break;
            }
        }

        for (auto &neighbor: get_neighbors(G,curr)) {
            if (G.weights[neighbor][4]<=1 && !visited[neighbor]) {
                visited[neighbor]=true;
                dists[neighbor]=dists[curr]+1;
                Q.push(neighbor);
            }
        }
    }

    // the timestep limit: no arrival before
    for (auto & p: approximate_goals) {
        p.second=max(goal.second-p.second,1);
    }

}

bool WHCAStar::run(const vector<State>& starts,
                   const vector< vector<pair<int, int> > >& goal_locations,
                   int time_limit)
{
    // set timer
    clock_t start = std::clock();
    num_expanded = 0;
    num_generated = 0;
    num_restarts = 0;
    int num_of_agents = starts.size();

    ReservationTable rt(G);
    rt.num_of_agents = num_of_agents;
    rt.map_size = G.size();
    rt.k_robust = k_robust;
    rt.window = window;
	rt.hold_endpoints = hold_endpoints;
    // path_planner.window = window;
    rt.use_cat = false;
    rt.prioritize_start = false;
    path_planner.prioritize_start = false;
	path_planner.hold_endpoints = hold_endpoints;
    path_planner.travel_times.clear();

    std::vector<int> priorities(num_of_agents);
    for (int i = 0; i < num_of_agents; i++)
        priorities[i] = i;

    runtime = (std::clock() - start) * 1.0  / CLOCKS_PER_SEC;
    while (runtime < time_limit)
    {
        num_restarts++;
        // cerr<<"restart: "<<num_restarts<<endl;
        // generate random priority order
        std::random_shuffle(priorities.begin(), priorities.end());

        solution_cost = 0;
        // solution = initial_solution;
        solution.clear();
        solution.resize(num_of_agents);
        bool succ = true;

        for (int idx=0;idx<priorities.size();++idx)
        {
            auto i= priorities[idx];
            // cerr<<"plan for agent "<<i<<" from "<<starts[i]<<" to goal:"<<goal_locations[i][0].first<<","<<goal_locations[i][0].second<<endl;
			rt.copy(initial_rt);
            rt.build(solution, initial_constraints, i);
            
            unordered_set<int> reservered_locations;
            for (int jdx=idx+1;jdx<priorities.size();++jdx)
            {
                auto j = priorities[jdx];
                rt.ct[starts[j].location].emplace_back(0,INT_MAX);
                reservered_locations.insert(starts[j].location);
            }

            auto & goal_location=goal_locations[i][0];
            unordered_map<int,int> approximate_goals;
            build_approximate_goals(approximate_goals, goal_location, reservered_locations);

            // cerr<<"approximate goals for goal "<<goal_location.first<<","<<goal_location.second<<":"<<endl;
            // for (auto & p: approximate_goals) {
            //     cerr<<p.first<<" "<<p.second<<"; ";
            // }
            // cerr<<endl;

			solution[i] = path_planner.run(G, starts[i], goal_locations[i], rt, approximate_goals);
            solution_cost += path_planner.path_cost;

            num_expanded += path_planner.num_expanded;
            num_generated += path_planner.num_generated;
            runtime = (std::clock() - start) * 1.0  / CLOCKS_PER_SEC;

            if (solution[i].size()==1 ) {
                solution[i].emplace_back(starts[i].wait());
//                 cerr<<"who fails sol? ["<<idx<<"] agent "<<i<<" "<<solution[i].size()<<" "<<runtime<<" "<<time_limit<<" "<<path_planner.num_expanded<<endl;
//                 cerr<<"approximate goals for goal "<<goal_location.first<<","<<goal_location.second<<" from "<<starts[i]<<":"<<endl;
//                 for (auto & p: approximate_goals) {
//                     cerr<<p.first<<" "<<p.second<<"; ";
//                 }
//                 cerr<<endl;
// cerr<<"reservation table:";
//                 for (auto & p: rt.ct) {
//                     cerr<<p.first<<":";
//                     for (auto & q: p.second) {
//                         cerr<<"("<<q.first<<","<<q.second<<"),";
//                     }
//                     cerr<<";"<<endl;
//                 }
//                 cerr<<endl;
//                 exit(-1);                
            }




            if (solution[i].empty() || runtime >= time_limit)
            {
                if (true) {
                cerr<<"who fails? ["<<idx<<"] agent "<<i<<" "<<solution[i].size()<<" "<<runtime<<" "<<time_limit<<" "<<path_planner.num_expanded<<endl;
                cerr<<"approximate goals for goal "<<goal_location.first<<","<<goal_location.second<<":"<<endl;
                for (auto & p: approximate_goals) {
                    cerr<<p.first<<" "<<p.second<<"; ";
                }
                cerr<<endl;
                cerr<<"reservation table:";
                for (auto & p: rt.ct) {
                    cerr<<p.first<<":";
                    for (auto & q: p.second) {
                        cerr<<"("<<q.first<<","<<q.second<<"),";
                    }
                    cerr<<";"<<endl;
                }
                cerr<<endl;
                }
                succ = false;
                rt.clear();
                break;
            }
            rt.clear();
        }
        if (succ)
        {
            runtime = (std::clock() - start) * 1.0  / CLOCKS_PER_SEC;
            min_sum_of_costs = 0;
            for (int i = 0; i < num_of_agents; i++)
            {
                int start = starts[i].location;
                for (const auto& goal : goal_locations[i])
                {
                    min_sum_of_costs += G.heuristics.at(goal.first)[start];
                    start = goal.first;
                }
            }
            avg_path_length = 0;
            for (int k = 0; k < num_of_agents; k++)
            {
                avg_path_length += (int)solution[k].size();
            }
            avg_path_length /= num_of_agents;
            solution_found = true;
            print_results();
            return true;
        }
    }
    runtime = (std::clock() - start) * 1.0  / CLOCKS_PER_SEC;
    solution_cost = -1;
    solution_found = false;
    print_results();
    return false;
}

void WHCAStar::print_results() const
{
    std::cout << "WHCA*:";
    if(solution_cost >= 0) // solved
        std::cout << "Succeed,";
    else // time_out
        std::cout << "Timeout,";

    std::cout << runtime << "," <<
                num_restarts << "," <<
                num_expanded << "," << num_generated << "," <<
                solution_cost << "," << min_sum_of_costs << "," <<
                avg_path_length <<
    std::endl;
}


void WHCAStar::save_results(const std::string &fileName, const std::string &instanceName) const
{
    std::ofstream stats;
    stats.open(fileName, std::ios::app);
    stats << runtime << "," <<
            num_restarts << "," << num_restarts << "," <<
            num_expanded << "," << num_generated << "," <<
            solution_cost << "," << min_sum_of_costs << "," <<
            avg_path_length << "," << "0" << "," <<
            instanceName << std::endl;
    stats.close();
}


void WHCAStar::clear()
{
	runtime = 0;
	solution_found = false;
	solution_cost = -2;
	avg_path_length = -1;
	num_expanded = 0;
	num_generated = 0;
	num_restarts = 0;
	solution.clear();
	initial_constraints.clear();
	initial_rt.clear();
}
}
