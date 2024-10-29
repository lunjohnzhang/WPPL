#include "CompetitionSystem.h"
#include "common.h"

class SortationSystem : public BaseSystem
{
public:
    SortationSystem(Grid &grid,
                    MAPFPlanner *planner,
                    ActionModelWithRotate *model,
                    // from destination to chutes
                    std::map<int, vector<int>> chute_mapping,
                    std::string package_mode,
                    // list of packages, essentially list of destinations
                    vector<int> packages,
                    // distribution of packages on each destination
                    vector<double> package_dist_weight,
                    // task assignment cost
                    std::string task_assignment_cost,
                    vector<double> task_assignment_params,
                    int num_agents, uint seed) : BaseSystem(grid, planner, model), MT(seed), task_id(0), chute_mapping(chute_mapping), package_mode(package_mode), packages(packages), package_dist_weight(package_dist_weight), task_assignment_cost(task_assignment_cost),
                        task_assignment_params(task_assignment_params)
    {
        num_of_agents = num_agents;
        starts.resize(num_of_agents);

        // Initial states of the agents
        std::shuffle(grid.empty_locations.begin(), grid.empty_locations.end(), MT);

        for (size_t i = 0; i < num_of_agents; i++)
        {
            starts[i] = State(grid.empty_locations[i], 0, -1);
            prev_task_locs.push_back(grid.empty_locations[i]);
        }

        // Initialize agent home location (workstation) distribution
        this->agent_home_loc_dist = std::discrete_distribution<int>(
            grid.agent_home_loc_weights.begin(),
            grid.agent_home_loc_weights.end());
        cout << "agent_home_loc distribution: ";
        for (auto w : grid.agent_home_loc_weights)
        {
            cout << w << ", ";
        }
        cout << endl;

        // Initialize package distribution
        if (package_mode == "dist")
        {
            this->package_dist = std::discrete_distribution<int>(
                package_dist_weight.begin(),
                package_dist_weight.end());
            cout << "package distribution: ";
            for (auto w : package_dist_weight)
            {
                cout << w << ", ";
            }
            cout << endl;
        }

        // Initialize number of robots going to each workstation
        for (auto workstation : grid.agent_home_locations)
        {
            robots_in_workstations[workstation] = 0;
        }

        // Initialize number of robots going to each endpoint
        for (auto endpoint : grid.end_points)
        {
            robots_in_endpoints[endpoint] = 0;
        }

        // Shuffle the packages
        std::shuffle(this->packages.begin(), this->packages.end(), MT);
    };

    void simulate(int simulation_time);

private:
    std::mt19937 MT;
    int task_id = 0;
    int package_id = 0;
    std::string task_assignment_cost;

    void update_tasks();
    std::discrete_distribution<int> agent_home_loc_dist;
    std::discrete_distribution<int> package_dist;

    std::string package_mode;
    std::vector<int> prev_task_locs;
    std::vector<int> packages;
    std::vector<double> package_dist_weight;
    std::map<int, vector<int>> chute_mapping;

    // workstations and #robots that intends to go to this workstation
    boost::unordered_map<int, int> robots_in_workstations;
    // endpoints and #robots that intends to go to this endpoint
    boost::unordered_map<int, int> robots_in_endpoints;

    double assign_C = 8;
    std::vector<double> task_assignment_params;
    double compute_assignment_cost(
        int curr_loc, pair<int, int> workstation) const;
    int assign_workstation(int curr_loc) const;
    int assign_endpoint(int curr_loc, vector<int> endpoints) const;
};
