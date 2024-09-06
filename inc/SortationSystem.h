#include "CompetitionSystem.h"
#include "common.h"

class SortationSystem : public BaseSystem
{
public:
    SortationSystem(Grid &grid,
                    MAPFPlanner *planner,
                    ActionModelWithRotate *model,
                    // from destination to chutes
                    std::map<int, int> chute_mapping,
                    std::string package_mode,
                    // list of packages, essentially list of destinations
                    vector<int> packages,
                    // distribution of packages on each destination
                    vector<double> package_dist_weight,
                    int num_agents, uint seed) : BaseSystem(grid, planner, model), MT(seed), task_id(0), chute_mapping(chute_mapping), package_mode(package_mode), packages(packages), package_dist_weight(package_dist_weight)
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


        // Shuffle the packages
        std::shuffle(this->packages.begin(), this->packages.end(), MT);
    };

private:
    std::mt19937 MT;
    int task_id = 0;
    int package_id = 0;

    void update_tasks();
    std::discrete_distribution<int> agent_home_loc_dist;
    std::discrete_distribution<int> package_dist;

    std::string package_mode;
    std::vector<int> prev_task_locs;
    std::vector<int> packages;
    std::vector<double> package_dist_weight;
    std::map<int, int> chute_mapping;
};
