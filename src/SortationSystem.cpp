#include "SortationSystem.h"
#include "util/Analyzer.h"

// next_pos: (next_loc, num_robots_targeting_next_loc)
double SortationSystem::compute_assignment_cost(
    int curr_loc,
    int next_loc,
    int robot_in_next_loc,
    int package_in_chute) const
{
    double cost = -1;
    if (this->task_assignment_cost == "heuristic+num_agents")
    {
        cost = this->planner->heuristics->get(curr_loc, next_loc) +
               this->assign_C * robot_in_next_loc;
    }
    else if (this->task_assignment_cost == "heuristic+num_packages")
    {
        if (this->map.grid_types[next_loc] == 'w')
        {
            cost = this->planner->heuristics->get(curr_loc, next_loc) +
                   this->assign_C * robot_in_next_loc;
        }
        // Use number of packages in the chute while going to endpionts
        else if (this->map.grid_types[next_loc] == 'e')
        {
            cost = this->planner->heuristics->get(curr_loc, next_loc) +
                   this->assign_C * package_in_chute;
        }
        else
        {
            std::cout << "Compute assignment cost: unknown grid type"
                      << std::endl;
            exit(-1);
        }
    }
    else if (this->task_assignment_cost == "heuristic")
    {
        cost = this->planner->heuristics->get(curr_loc, next_loc);
    }
    else if (this->task_assignment_cost == "num_agents")
    {
        cost = robot_in_next_loc;
    }
    else if (this->task_assignment_cost == "opt_quadratic_f")
    {
        // Cost is computed by a function
        // f(x, y) = a_0 * x^2 + a_1 y^2 + a_2 * xy + a_3* x + a_4 * y + a_5,
        // where x and y are heuristic and number of agents, respectively, and
        // a's are optimized parameters
        // Use different sets of parameters for different targets
        double x = this->planner->heuristics->get(curr_loc, next_loc);
        double y = robot_in_next_loc;
        if (this->map.grid_types[next_loc] == 'w')
        {
            cost = this->task_assignment_params[0] * x * x +
                   this->task_assignment_params[1] * y * y +
                   this->task_assignment_params[2] * x * y +
                   this->task_assignment_params[3] * x +
                   this->task_assignment_params[4] * y;
        }
        else if (this->map.grid_types[next_loc] == 'e')
        {
            cost = this->task_assignment_params[5] * x * x +
                   this->task_assignment_params[6] * y * y +
                   this->task_assignment_params[7] * x * y +
                   this->task_assignment_params[8] * x +
                   this->task_assignment_params[9] * y;
        }
        else
        {
            std::cout << "Compute assignment cost: unknown grid type"
                      << std::endl;
            exit(-1);
        }
    }
    else
    {
        std::cout << "unknown task assignment strategy" << std::endl;
        exit(-1);
    }
    return cost;
}

int SortationSystem::assign_workstation(int curr_loc)
{
    // Assign random workstation if task assignment is random
    if (this->task_assignment_cost == "random")
    {
        int idx = MT() % this->map.agent_home_locations.size();
        return this->map.agent_home_locations[idx];
    }

    // Choose a workstation based on:
    // heuristic[curr][next] + C * #robots targeting `next`
    int assigned_loc;
    double min_cost = DBL_MAX;

    ONLYDEV(
        int x = curr_loc % map.cols;
        int y = curr_loc / map.cols;
        cout << "current loc: (" << x << ", " << y << ")" << endl;)

    for (auto workstation : this->robots_in_workstations)
    {
        // double cost = this->planner->heuristics->get(curr_loc, workstation.first) + this->assign_C * workstation.second;
        double cost = this->compute_assignment_cost(
            curr_loc, workstation.first, workstation.second, -1);
        if (cost < min_cost)
        {
            min_cost = cost;
            assigned_loc = workstation.first;
        }
    }
    ONLYDEV(
        x = assigned_loc % map.cols;
        y = assigned_loc / map.cols;
        cout << "assigned workstation: (" << x << ", " << y << ")" << endl;)
    return assigned_loc;
}

pair<int, int> SortationSystem::assign_endpoint(
    int curr_loc, vector<pair<int, int>> endpoints)
{
    // Assign random workstation if task assignment is random
    if (this->task_assignment_cost == "random")
    {
        int idx = MT() % endpoints.size();
        return endpoints[idx];
    }

    // Choose an endpoint based on:
    // heuristic[curr][next] + C * #robots targeting `next`
    int assigned_loc;
    int assigned_chute;
    double min_cost = DBL_MAX;

    for (auto ele : endpoints)
    {
        int endpoint = ele.first;
        int chute = ele.second;
        // double cost = this->planner->heuristics->get(curr_loc, endpoint) + this->assign_C * this->robots_in_endpoints.at(endpoint);
        double cost = this->compute_assignment_cost(
            curr_loc, endpoint,
            this->robots_in_endpoints.at(endpoint),
            this->packages_in_chutes[chute]);
        if (cost < min_cost)
        {
            min_cost = cost;
            assigned_loc = endpoint;
            assigned_chute = chute;
        }
    }
    return make_pair(assigned_loc, assigned_chute);
}

void SortationSystem::create_task_distribution(vector<double> dist)
{
    if (package_mode == "dist")
    {
        if (this->recirc_mechanism)
        {
            this->package_dist = std::discrete_distribution<int>(
                dist.begin(),
                dist.end() - 1);
            this->recir_chute = chute_mapping[
                dist.size() - 1][0];
        }
        else
        {
            this->package_dist = std::discrete_distribution<int>(
                dist.begin(),
                dist.end());
        }
        // cout << "Update package distribution: ";
        // for (auto w : dist)
        // {
        //     cout << w << ", ";
        // }
        // cout << endl;
    }
}

void SortationSystem::update_task_distribution()
{
    if (this->time_dist)
    {
        if (this->timestep < this->total_simulation_steps)
        {
            // Update the package distribution
            this->create_task_distribution(
                this->time_package_dist_weight[this->timestep]);
        }
    }
    else
    {
        // Sample gaussian noise
        for (int i = 0; i < this->package_dist_weight.size(); i++)
        {
            // Sample gaussian noise
            double sigma;
            // If the task_gaussian_sigma is not set, use half of the weight as
            // the sigma, make sure the resulting weight has low probability of
            // being
            // negative
            if (this->task_gaussian_sigma == -1)
            {
                sigma = this->init_package_dist_weight[i] / 2.0;
            }
            else
            {
                sigma = this->task_gaussian_sigma;
            }
            std::normal_distribution<double> dist(0, sigma);
            // Add gaussian noise to the package distribution weights
            this->package_dist_weight[i] = this->init_package_dist_weight[i] +
                                           dist(this->MT);
            if (this->package_dist_weight[i] < 0)
            {
                this->package_dist_weight[i] = 0;
            }
        }
        // Update the package distribution
        this->create_task_distribution(this->package_dist_weight);
    }

}


void SortationSystem::update_tasks()
{
    // Update task distribution
    if (this->timestep % this->task_change_time == 0)
    {
        this->update_task_distribution();
    }

    for (int k = 0; k < num_of_agents; k++)
    {
        while (assigned_tasks[k].size() < num_tasks_reveal)
        {
            int prev_task_loc = prev_task_locs[k];
            Task prev_task = prev_tasks[k];

            int loc;
            bool recirc = false;
            int next_package = -1;
            int assigned_chute = -1;
            int dummy_waiting = false;

            // Agent is still waiting in the previous task location, assign the
            // previous goal as the dummy goal
            if (this->agent_task_waiting[k])
            {
                dummy_waiting = true;
                loc = this->agent_task_waiting_loc[k];
                recirc = prev_task.recirc;
                next_package = prev_task.package_dest;
                assigned_chute = prev_task.assigned_chute;
            }
            // Agent is not waiting, assign the next goal
            else
            {
                if (map.grid_types[prev_task_loc] == '.' ||
                    map.grid_types[prev_task_loc] == 'e')
                {
                    // next task would be w
                    // int idx = this->agent_home_loc_dist(this->MT);
                    // int idx=MT()%map.agent_home_locations.size();
                    // loc = map.agent_home_locations[idx];
                    loc = assign_workstation(prev_task_loc);
                    this->robots_in_workstations[loc]++;
                    // cout << "Agent " << k << " Assigned workstation: " << loc << endl;
                }
                else if (map.grid_types[prev_task_loc] == 'w')
                {
                    // next task would be an endpoint determined by the packages
                    // and chute mapping
                    // First check if there are any recirculation packages
                    if (!this->recirc_packages.empty())
                    {
                        next_package = this->recirc_packages.front();
                        this->recirc_packages.pop();
                        // cout << "Recirculation package " << next_package << endl;
                        // cout << "Recirculation package size: " << this->recirc_packages.size() << endl;
                    }
                    else
                    {
                        // Sample new package
                        if (this->package_mode == "explicit")
                        {
                            if (this->package_id >= this->packages.size())
                            {
                                std::cout << "not enough packages" << std::endl;
                                exit(-1);
                            }
                            next_package = this->packages[this->package_id];
                        }
                        else if (this->package_mode == "dist")
                        {
                            next_package = this->package_dist(this->MT);
                        }
                        else
                        {
                            std::cout << "unkonw package mode" << std::endl;
                            exit(-1);
                        }

                        this->package_id++;
                    }

                    // int chute_idx = MT() % this->chute_mapping[next_package].size();
                    // int next_chute = this->chute_mapping[next_package][chute_idx];
                    // // Choose a random endpoint around the mapped chute
                    // int endpt_idx = MT() % map.obs_adj_endpoints[next_chute].size();
                    // loc = map.obs_adj_endpoints[next_chute][endpt_idx];

                    // Get all endpoints around the chutes being mapped to
                    // pair of endpoint and the corresponding chute
                    vector<pair<int, int>> endpoints;
                    for (auto chute : this->chute_mapping[next_package])
                    {
                        if (!this->chute_sleeping[chute])
                        {
                            for (auto endpoint : map.obs_adj_endpoints[chute])
                            {
                                endpoints.push_back(
                                    std::make_pair(endpoint, chute));
                            }
                        }
                    }
                    // All chutes are sleeping, go to the recirculation chute
                    if (endpoints.empty())
                    {
                        if (!this->recirc_mechanism)
                        {
                            std::cout << "All chutes are sleeping, but recirculation mechanism is disabled"
                                      << std::endl;
                            exit(-1);
                        }
                        ONLYDEV(cout << "Agent " << k
                                     << " going to recirculation chute "
                                     << this->recir_chute << endl;)

                        recirc = true;
                        for (auto endpoint : map.obs_adj_endpoints[this->recir_chute])
                        {
                            ONLYDEV(cout << "Endpoint: " << endpoint << endl;)
                            endpoints.push_back(
                                std::make_pair(endpoint, this->recir_chute));
                        }
                    }

                    // Choose the next endpoint
                    std::tie(loc, assigned_chute) = assign_endpoint(
                        prev_task_loc, endpoints);
                    this->robots_in_endpoints[loc]++;
                    ONLYDEV(if (recirc) {
                        cout << "Agent " << k << " Assigned endpoint: " << loc
                             << " (chute: " << assigned_chute << ")" << endl;
                    })
                }
                else
                {
                    std::cout << "unkonw grid type" << std::endl;
                    exit(-1);
                }
            }

            Task task(task_id, loc, timestep, k, recirc, next_package,
                      assigned_chute, dummy_waiting);
            assigned_tasks[k].push_back(task);
            events[k].push_back(make_tuple(task.task_id, timestep, "assigned"));
            log_event_assigned(k, task.task_id, timestep);
            all_tasks.push_back(task);
            prev_task_locs[k] = loc;
            prev_tasks[k] = task;
            task_id++;
        }
    }
}

void SortationSystem::simulate(int simulation_time)
{
    // init logger
    // Logger* log = new Logger();
    // cout << "Sorting Simulation starts!" << std::endl;
    ONLYDEV(g_timer.record_p("simulate_start");)

    initialize();

    ONLYDEV(g_timer.record_d("simulate_start", "initialize_end", "initialization");)
    // int num_of_tasks = 0;

    for (; timestep < simulation_time;)
    {
        ONLYDEV(
            cout << "----------------------------" << std::endl;
            cout << "Timestep " << timestep << std::endl;)

        // find a plan
        sync_shared_env();
        // vector<Action> actions = planner->plan(plan_time_limit);
        // vector<Action> actions;
        // planner->plan(plan_time_limit,actions);

        auto start = std::chrono::steady_clock::now();

        vector<Action> actions = plan();

        ONLYDEV(
            if (actions.size() == num_of_agents) {
                analyzer.data["moving_steps"] = analyzer.data["moving_steps"].get<int>() + 1;
            } else {
                if (actions.size() != 0)
                {
                    DEV_DEBUG("planner return wrong number of actions: {}", actions.size());
                    exit(-1);
                }
                else
                {
                    DEV_WARN("planner return no actions: most likely exceeding the time limit.");
                }
            })

        auto end = std::chrono::steady_clock::now();

        timestep += 1;
        ONLYDEV(analyzer.data["timesteps"] = timestep;)

        for (int a = 0; a < num_of_agents; a++)
        {
            if (!env->goal_locations[a].empty())
                solution_costs[a]++;
        }

        // move drives
        list<Task> new_finished_tasks = move(actions);
        if (!planner_movements[0].empty() && planner_movements[0].back() == Action::NA)
        {
            planner_times.back() += plan_time_limit; // add planning time to last record
        }
        else
        {
            auto diff = end - start;
            planner_times.push_back(std::chrono::duration<double>(diff).count());
        }
        ONLYDEV(cout << new_finished_tasks.size() << " tasks has been finished in this timestep" << std::endl;)

        // update tasks
        for (auto task : new_finished_tasks)
        {
            bool finish_waiting = this->update_task_status(task);
            if (finish_waiting)
            {
                this->process_finished_task_offline(task);
            }
        }
        ONLYDEV(cout << this->n_finish_task_plus_n_recirs << " tasks has been finished by far in total" << std::endl;)

        ONLYDEV(analyzer.data["finished_tasks"] = this->n_finish_task_plus_n_recirs;)

        // Put chutes to sleep if they are full
        if (this->recirc_mechanism)
        {
            update_chute_sleeping();
            // if (timestep % 500 == 0)
            // print_chute_sleeping_status();
        }

        // Generate new tasks
        update_tasks();

        ONLYDEV(check_n_agents_sum();)

        bool complete_all = false;
        for (auto &t : assigned_tasks)
        {
            if (t.empty())
            {
                complete_all = true;
            }
            else
            {
                complete_all = false;
                break;
            }
        }
        if (complete_all)
        {
            cout << std::endl
                 << "All task finished!" << std::endl;
            break;
        }

        ONLYDEV(g_timer.print_all_d(););
    }
    ONLYDEV(g_timer.record_d("initialize_end", "simulate_end", "simulation");)

    ONLYDEV(g_timer.print_all_d();)

    cout << std::endl
         << "Done!" << std::endl;
    // cout << num_of_tasks << " tasks has been finished by far in total" << std::endl;

    ONLYDEV(analyzer.dump();)
}

void SortationSystem::warmup(int total_warmup_steps)
{
    initialize();
    for (; this->warmupstep < total_warmup_steps;)
    {
        sync_shared_env();
        auto start = std::chrono::steady_clock::now();
        vector<Action> actions = plan();
        auto end = std::chrono::steady_clock::now();
        this->warmupstep += 1;

        for (int a = 0; a < num_of_agents; a++)
        {
            if (!env->goal_locations[a].empty())
                solution_costs[a]++;
        }

        // move drives
        list<Task> new_finished_tasks = move(actions);
        if (!planner_movements[0].empty() && planner_movements[0].back() == Action::NA)
        {
            planner_times.back() += plan_time_limit; // add planning time to last record
        }
        else
        {
            auto diff = end - start;
            planner_times.push_back(std::chrono::duration<double>(diff).count());
        }

        // TODO: ensure warmup steps does not contribute to the whole setting
        this->curr_finish_task_agents.clear();
        for (auto task : new_finished_tasks)
        {
            bool finish_waiting = this->update_task_status(task);
            if (finish_waiting)
            {
                this->process_finished_task_online(task, true);
            }

        }

        // Put chutes to sleep if they are full
        if (this->recirc_mechanism)
        {
            update_chute_sleeping();
        }

        update_tasks();
        ONLYDEV(check_n_agents_sum();)

        bool complete_all = false;
        for (auto &t : assigned_tasks)
        {
            if (t.empty())
            {
                complete_all = true;
            }
            else
            {
                complete_all = false;
                break;
            }
        }
        if (complete_all)
        {
            cout << std::endl
                 << "All task finished!" << std::endl;
            break;
        }
    }
}

int SortationSystem::update_gg_and_step(int update_gg_interval)
{
    this->curr_starts = this->curr_states;
    // cout << "Sortation system: update_gg_and_step" << endl;

    int t_step = 0;
    for (; t_step < update_gg_interval && this->timestep < this->total_simulation_steps;)
    {
        sync_shared_env();
        auto start = std::chrono::steady_clock::now();
        vector<Action> actions = plan();
        auto end = std::chrono::steady_clock::now();
        t_step += 1;
        timestep += 1;

        for (int a = 0; a < num_of_agents; a++)
        {
            if (!env->goal_locations[a].empty())
                solution_costs[a]++;
        }

        // move drives
        list<Task> new_finished_tasks = move(actions);
        if (!planner_movements[0].empty() && planner_movements[0].back() == Action::NA)
        {
            planner_times.back() += plan_time_limit; // add planning time to last record
        }
        else
        {
            auto diff = end - start;
            planner_times.push_back(std::chrono::duration<double>(diff).count());
        }

        this->curr_finish_task_agents.clear();
        for (auto task : new_finished_tasks)
        {
            bool finish_waiting = this->update_task_status(task);
            if (finish_waiting)
            {
                this->process_finished_task_online(task, false);
            }
        }

        // // Print content of robots_in_workstations and robots_in_endpoints
        // cout << "robots_in_workstations: ";
        // for (auto & r: robots_in_workstations){
        //     cout << r.second << ", ";
        // }
        // cout << endl;
        // cout << "robots_in_endpoints: ";
        // for (auto & r: robots_in_endpoints){
        //     cout << r.second << ", ";
        // }
        // cout << endl;

        // if (this->task_dist_change_interval>0 && this->timestep%this->task_dist_change_interval == 0){
        //     this->random_update_tasks_distribution();
        // }

        // Put chutes to sleep if they are full
        if (this->recirc_mechanism)
        {
            update_chute_sleeping();
            // print_chute_sleeping_status();
        }

        // Generate new tasks
        update_tasks();

        ONLYDEV(check_n_agents_sum();)

        // Sum of robots in robots_in_workstations and robots_in_endpoints must be equal to num_of_agents

        bool complete_all = false;
        for (auto &t : assigned_tasks)
        {
            if (t.empty())
            {
                complete_all = true;
            }
            else
            {
                complete_all = false;
                break;
            }
        }
        if (complete_all)
        {
            cout << std::endl
                 << "All task finished!" << std::endl;
            break;
        }
    }
    return t_step;
    // exit(1);
}

void SortationSystem::update_n_agents(Task task)
{
    if (map.grid_types[task.location] == 'w')
    {
        this->robots_in_workstations[task.location]--;
    }
    else if (map.grid_types[task.location] == 'e')
    {
        this->robots_in_endpoints[task.location]--;
    }
}

void SortationSystem::check_n_agents_sum()
{
    int sum = 0;
    for (auto &r : robots_in_workstations)
    {
        sum += r.second;
    }
    for (auto &r : robots_in_endpoints)
    {
        sum += r.second;
    }
    if (sum != num_of_agents)
    {
        cout << "Sum of robots in workstations and endpoints is not equal to num_of_agents" << endl;
        exit(-1);
    }
    else
    {
        cout << "Sum of robots in workstations and endpoints is equal to num_of_agents" << endl;
    }
}

void SortationSystem::update_chute_sleeping()
{
    for (auto &chute : this->chute_sleeping)
    {
        // Increment sleeping time of already sleeping chutes
        if (chute.second)
        {
            this->chute_sleeping_time[chute.first]++;
            if (this->chute_sleeping_time[chute.first] >= CHUTE_SLEEP_TIME)
            {
                this->chute_sleeping[chute.first] = false;
                this->chute_sleeping_time[chute.first] = 0;
                this->packages_in_chutes[chute.first] = 0;
            }
        }
        // Put chutes to sleep if they have too many packages
        else
        {
            // Recirculation chute never sleep
            if (chute.first != this->recir_chute &&
                this->packages_in_chutes[chute.first] >= MAX_PACKAGE_IN_CHUTE)
            {
                this->chute_sleeping[chute.first] = true;
                this->chute_sleep_count[chute.first]++;
            }
        }
    }
}

void SortationSystem::print_chute_sleeping_status()
{
    // Print n packages in chute
    cout << "Packages in chutes: ";
    for (auto &chute : this->packages_in_chutes)
    {
        cout << chute.first << ": " << chute.second << " ";
        // cout << "Chute " << chute.first << ": " << chute.second << " packages" << endl;
    }
    cout << endl;

    // cout << "Chute sleeping status: " << endl;
    for (auto &chute : this->chute_sleeping)
    {
        if (chute.second)
        {
            cout << "Chute " << chute.first << ": " << "sleeping for " << this->chute_sleeping_time[chute.first] << " timesteps" << endl;
        }
    }
}

void SortationSystem::update_package_in_chute(Task task)
{
    if (task.assigned_chute != -1)
    {
        this->packages_in_chutes[task.assigned_chute]++;
    }
}

// Update the task waiting status of the agents.
// Return true of the corresponding agent has waited for enough time for the
// task to be processed
bool SortationSystem::update_task_status(Task task)
{
    int waiting_time = this->task_waiting_time;
    if (this->map.grid_types[task.location] == 'w')
    {
        waiting_time = this->workstation_waiting_time;
    }

    // No need to wait
    if (waiting_time == 0)
    {
        return true;
    }

    // Current task is a regular task, then the agent should starting waiting
    // on the task
    int ag = task.agent_assigned;
    if (!task.dummy_waiting)
    {
        ONLYDEV(cout << "t=" << timestep << " Agent " << ag
                     << " starts waiting for the task " << task.location
                     << endl;)
        this->agent_task_waiting[ag] = true;
        this->agent_task_waiting_loc[ag] = task.location;
        this->agent_task_waiting_time[ag] = 1;
        // Add the agent to the task waiting list
        this->task_wait_agents.insert(ag);
        return false;
    }
    // If the waiting has not finished, increment the corresponding waiting
    // status
    else if (task.dummy_waiting &&
             this->agent_task_waiting_time[ag] < waiting_time)
    {
        ONLYDEV(cout << "t=" << timestep << " Agent " << ag
                     << " is waiting for the task " << task.location << endl;)
        this->agent_task_waiting_time[ag]++;
        return false;
    }
    // Waiting has finished, add the task as finished task
    else if (task.dummy_waiting &&
             this->agent_task_waiting_time[ag] >= waiting_time)
    {
        ONLYDEV(cout << "t=" << timestep << " Agent " << ag
                     << " ends waiting for the task " << task.location << endl;)
        this->agent_task_waiting[ag] = false;
        this->agent_task_waiting_loc[ag] = -1;
        this->agent_task_waiting_time[ag] = 0;
        // Remove the agent from task waiting list
        if (this->task_wait_agents.find(ag) != this->task_wait_agents.end())
        {
            this->task_wait_agents.erase(ag);
        }
        else
        {
            cout << "Agent " << ag << " is not in the task waiting list"
                 << endl;
            exit(-1);
        }
        return true;

    }
    else
    {
        cout << "Unknown task waiting status" << endl;
        exit(-1);
    }
}

void SortationSystem::process_finished_task_offline(Task task)
{
    // Count finished tasks if the task is not recirculation
    if (!task.recirc)
    {
        num_of_task_finish++;
        finished_tasks[task.agent_assigned].emplace_back(task);
    }
    else
    {
        this->n_recirs++;
        ONLYDEV(cout << "Adding recirculation package "
                        << task.package_dest
                        << " finished" << std::endl;)
        this->recirc_packages.push(task.package_dest);
    }
    ONLYDEV(cout << "Agent " << task.agent_assigned
                    << " finished task " << task.location
                    << " for chute " << task.assigned_chute
                    << " at timestep " << timestep << endl;)

    this->n_finish_task_plus_n_recirs++;
    // decrement the number of robots going to the goals
    this->update_n_agents(task);
    // update the number of packages in the chute
    this->update_package_in_chute(task);
}

void SortationSystem::process_finished_task_online(Task task, bool warmup)
{
    if (warmup)
    {
        if (!task.recirc)
        {
            finished_tasks[task.agent_assigned].emplace_back(task);
            this->curr_finish_task_agents.push_back(task.agent_assigned);
            // Tasks finished during warm are not counted
            // num_of_task_finish++;
        }
        else
        {
            // Recircs during warmup are not counted, but the packages are
            // still recirculated
            // this->n_recirs++;
            this->recirc_packages.push(task.package_dest);
        }
    }
    else
    {
        this->n_finish_task_plus_n_recirs++;
        if (!task.recirc)
        {
            this->curr_finish_task_agents.push_back(task.agent_assigned);
            num_of_task_finish++;
            finished_tasks[task.agent_assigned].emplace_back(task);
        }
        else
        {
            this->n_recirs++;
            this->recirc_packages.push(task.package_dest);
            // cout << "Adding recirculation package " << task.package_dest << " finished" << std::endl;
        }
    }
    // Decrement the number of robots in the workstation
    this->update_n_agents(task);

    // Update number of packages in the chute
    this->update_package_in_chute(task);
}


void SortationSystem::gen_time_dist(
    int n_destinations,
    const std::vector<double>& package_weight_dist,
    int time_sigma,
    int T
) {
    // Ensure the input weights match the number of destinations
    assert(static_cast<int>(package_weight_dist.size()) == n_destinations);

    // Random number generator
    // std::random_device rd;
    // std::mt19937 gen(rd());
    std::uniform_int_distribution<int> random_time(0, T - 1);

    // Container for the distribution over time
    this->time_package_dist_weight.resize(
        T, std::vector<double>(n_destinations, 0.0));
    // std::vector<std::vector<double>> dist_over_time(
    //     T, std::vector<double>(n_destinations, 0.0));

    // Generate a Gaussian for each destination
    for (int d = 0; d < n_destinations; ++d) {
        double volume = package_weight_dist[d];
        int t = random_time(this->MT); // Random time center

        // Create the Gaussian distribution
        std::vector<double> gaussian(T);
        double gaussian_sum = 0.0;
        for (int i = 0; i < T; ++i) {
            gaussian[i] = std::exp(-std::pow(i + 1 - t, 2) / (2.0 * std::pow(time_sigma, 2)));
            gaussian_sum += gaussian[i];
        }

        // Normalize the Gaussian to match the volume
        for (int i = 0; i < T; ++i) {
            this->time_package_dist_weight[i][d] = (gaussian[i] / gaussian_sum) * volume;
        }
    }
}