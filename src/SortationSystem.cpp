#include "SortationSystem.h"
#include "util/Analyzer.h"

double SortationSystem::compute_assignment_cost(
    int curr_loc, pair<int, int> workstation) const
{
    double cost = -1;
    if (this->task_assignment_cost == "heuristic+num_agents")
    {
        cost = this->planner->heuristics->get(curr_loc, workstation.first) + this->assign_C * workstation.second;
    }
    else if (this->task_assignment_cost == "heuristic")
    {
        cost = this->planner->heuristics->get(curr_loc, workstation.first);
    }
    else
    {
        std::cout << "unknown task assignment strategy" << std::endl;
        exit(-1);
    }
    return cost;
}

int SortationSystem::assign_workstation(int curr_loc) const
{
    // Choose a workstation based on:
    // heuristic[curr][next] + C * #robots targeting `next`
    int assigned_loc;
    double min_cost = DBL_MAX;

    ONLYDEV(
        int x = curr_loc % map.cols;
        int y = curr_loc / map.cols;
        cout << "current loc: (" << x << ", " << y << ")" << endl;
    )

    for (auto workstation : this->robots_in_workstations)
    {
        // double cost = this->planner->heuristics->get(curr_loc, workstation.first) + this->assign_C * workstation.second;
        double cost = this->compute_assignment_cost(curr_loc, workstation);
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

int SortationSystem::assign_endpoint(int curr_loc, vector<int> endpoints) const
{
    // Choose an endpoint based on:
    // heuristic[curr][next] + C * #robots targeting `next`
    int assigned_loc;
    double min_cost = DBL_MAX;

    for (auto endpoint : endpoints)
    {
        double cost = this->planner->heuristics->get(curr_loc, endpoint) + this->assign_C * this->robots_in_endpoints.at(endpoint);
        if (cost < min_cost)
        {
            min_cost = cost;
            assigned_loc = endpoint;
        }
    }
    return assigned_loc;
}

void SortationSystem::update_tasks()
{
    for (int k = 0; k < num_of_agents; k++)
    {
        while (assigned_tasks[k].size() < num_tasks_reveal)
        {
            int prev_task_loc = prev_task_locs[k];

            int loc;
            if (map.grid_types[prev_task_loc] == '.' ||
                map.grid_types[prev_task_loc] == 'e')
            {
                // next task would be w
                // int idx = this->agent_home_loc_dist(this->MT);
                // int idx=MT()%map.agent_home_locations.size();
                // loc = map.agent_home_locations[idx];
                loc = assign_workstation(prev_task_loc);
                this->robots_in_workstations[loc]++;
            }
            else if (map.grid_types[prev_task_loc] == 'w')
            {
                // next task would be an endpoint determined by the packages
                // and chute mapping
                int next_package;
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

                // int chute_idx = MT() % this->chute_mapping[next_package].size();
                // int next_chute = this->chute_mapping[next_package][chute_idx];
                // // Choose a random endpoint around the mapped chute
                // int endpt_idx = MT() % map.obs_adj_endpoints[next_chute].size();
                // loc = map.obs_adj_endpoints[next_chute][endpt_idx];

                // Get all endpoints around the chutes being mapped to
                vector<int> endpoints;
                for (auto chute : this->chute_mapping[next_package])
                {
                    for (auto endpoint : map.obs_adj_endpoints[chute])
                    {
                        endpoints.push_back(endpoint);
                    }
                }
                // Choose the next endpoint
                loc = assign_endpoint(prev_task_loc, endpoints);
                this->robots_in_endpoints[loc]++;
            }
            else
            {
                std::cout << "unkonw grid type" << std::endl;
                exit(-1);
            }

            Task task(task_id, loc, timestep, k);
            assigned_tasks[k].push_back(task);
            events[k].push_back(make_tuple(task.task_id, timestep, "assigned"));
            log_event_assigned(k, task.task_id, timestep);
            all_tasks.push_back(task);
            prev_task_locs[k] = loc;
            task_id++;
        }
    }
}

void SortationSystem::simulate(int simulation_time)
{
    // init logger
    // Logger* log = new Logger();

    ONLYDEV(g_timer.record_p("simulate_start");)

    initialize();

    ONLYDEV(g_timer.record_d("simulate_start", "initialize_end", "initialization");)
    int num_of_tasks = 0;

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
            // int id, loc, t;
            // std::tie(id, loc, t) = task;
            finished_tasks[task.agent_assigned].emplace_back(task);
            num_of_tasks++;
            num_of_task_finish++;

            // update the number of robots in the workstation
            if (map.grid_types[task.location] == 'w')
            {
                this->robots_in_workstations[task.location]--;
            }
            else if (map.grid_types[task.location] == 'e')
            {
                this->robots_in_endpoints[task.location]--;
            }
        }
        ONLYDEV(cout << num_of_tasks << " tasks has been finished by far in total" << std::endl;)

        ONLYDEV(analyzer.data["finished_tasks"] = num_of_tasks;)

        update_tasks();

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
    cout << num_of_tasks << " tasks has been finished by far in total" << std::endl;

    ONLYDEV(analyzer.dump();)
}