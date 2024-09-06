#include "SortationSystem.h"

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
                // Sample a workstation based on given distribution
                int idx = this->agent_home_loc_dist(this->MT);
                // int idx=MT()%map.agent_home_locations.size();
                loc = map.agent_home_locations[idx];
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
                int next_chute = this->chute_mapping[next_package];
                // Choose a random endpoint around the mapped chute
                int idx = MT() % map.obs_adj_endpoints[next_chute].size();
                loc = map.obs_adj_endpoints[next_chute][idx];
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