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
                    double assign_C,
                    bool recirc_mechanism,
                    int task_waiting_time,
                    int workstation_waiting_time,
                    int num_agents, uint seed) : BaseSystem(grid, planner, model), MT(seed), task_id(0), chute_mapping(chute_mapping), package_mode(package_mode), packages(packages), package_dist_weight(package_dist_weight), task_assignment_cost(task_assignment_cost),
                        task_assignment_params(task_assignment_params),
                        assign_C(assign_C), recirc_mechanism(recirc_mechanism),
                        task_waiting_time(task_waiting_time),
                        workstation_waiting_time(workstation_waiting_time)
    {
        num_of_agents = num_agents;
        starts.resize(num_of_agents);

        // Initial states of the agents
        std::shuffle(grid.empty_locations.begin(), grid.empty_locations.end(), MT);

        for (size_t i = 0; i < num_of_agents; i++)
        {
            int loc = grid.empty_locations[i];
            starts[i] = State(loc, 0, -1);
            prev_task_locs.push_back(loc);
            prev_tasks.push_back(Task(-1, loc));
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
            if (this->recirc_mechanism)
            {
                this->package_dist = std::discrete_distribution<int>(
                    package_dist_weight.begin(),
                    package_dist_weight.end() - 1);
                this->recir_chute = chute_mapping[
                    package_dist_weight.size() - 1][0];
            }
            else
            {
                this->package_dist = std::discrete_distribution<int>(
                    package_dist_weight.begin(),
                    package_dist_weight.end());
            }
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

        // Chute sleeping logic
        for (auto chute : grid.chutes)
        {
            this->chute_sleeping_time[chute] = 0;
            this->chute_sleeping[chute] = false;
            this->packages_in_chutes[chute] = 0;
            this->chute_sleep_count[chute] = 0;
        }

        // Task waiting mechanism
        for (int i = 0; i < num_of_agents; i++)
        {
            this->agent_task_waiting_time.push_back(0);
            this->agent_task_waiting.push_back(false);
            this->agent_task_waiting_loc.push_back(-1);
        }


    };

    void simulate(int simulation_time) override;
    void warmup(int total_warmup_steps) override;
    int update_gg_and_step(int update_gg_interval) override;
    int get_n_finish_task_plus_n_recirs() const
    {
        return n_finish_task_plus_n_recirs;
    }
    int get_n_recirs() const
    {
        return n_recirs;
    }
    boost::unordered_map<int, int> get_chute_sleep_count() const
    {
        return chute_sleep_count;
    }


private:
    std::mt19937 MT;
    int task_id = 0;
    int package_id = 0;
    std::string task_assignment_cost;

    void update_tasks() override;
    std::discrete_distribution<int> agent_home_loc_dist;
    std::discrete_distribution<int> package_dist;

    std::string package_mode;
    std::vector<int> prev_task_locs;
    std::vector<Task> prev_tasks;
    std::vector<int> packages;
    std::vector<double> package_dist_weight;
    std::map<int, vector<int>> chute_mapping;
    bool recirc_mechanism = true;
    int recir_chute = -1;
    int n_recirs = 0;
    int n_finish_task_plus_n_recirs = 0;
    queue<int> recirc_packages;

    // workstations and #robots that intends to go to this workstation
    boost::unordered_map<int, int> robots_in_workstations;
    // endpoints and #robots that intends to go to this endpoint
    boost::unordered_map<int, int> robots_in_endpoints;
    // number of packages in each chute
    boost::unordered_map<int, int> packages_in_chutes;
    // once exceeded, chute goes to sleep for 50 timesteps
    int MAX_PACKAGE_IN_CHUTE = 50;
    // sleeping time for chute
    int CHUTE_SLEEP_TIME = 50;
    // number of timesteps chute has been sleeping
    boost::unordered_map<int, int> chute_sleeping_time;
    // whether chute is sleeping
    boost::unordered_map<int, bool> chute_sleeping;
    // Number of times each chute has slept
    boost::unordered_map<int, int> chute_sleep_count;

    // Task waiting mechanism: Agents need to wait at their goal for a fixed
    // number of timesteps
    int task_waiting_time = 5;
    int workstation_waiting_time = 1;
    // Task waiting time counter
    vector<int> agent_task_waiting_time;
    // Task waiting status
    vector<bool> agent_task_waiting;
    // Task waiting location
    vector<int> agent_task_waiting_loc;

    double assign_C = 8;
    std::vector<double> task_assignment_params;
    double compute_assignment_cost(
        int curr_loc,
        int next_loc,
        int robot_in_next_loc,
        int package_in_chute) const;
    int assign_workstation(int curr_loc);
    pair<int, int> assign_endpoint(int curr_loc,
                                   vector<pair<int, int>> endpoints);
    void update_n_agents(Task task);
    void check_n_agents_sum();
    void update_chute_sleeping();
    void print_chute_sleeping_status();
    void update_package_in_chute(Task task);
    bool update_task_status(Task task);
    void process_finished_task_offline(Task task);
    void process_finished_task_online(Task task, bool warmup);
};
