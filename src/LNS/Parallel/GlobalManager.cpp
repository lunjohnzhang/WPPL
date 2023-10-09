#include "LNS/Parallel/GlobalManager.h"
#include "util/Timer.h"
#include "omp.h"
#include "LNS/Parallel/TimeLimiter.h"

namespace LNS {

GlobalManager::GlobalManager(
    Instance & instance, PathTable & path_table, std::vector<Agent> & agents, std::shared_ptr<HeuristicTable> HT,
    int neighbor_size, destroy_heuristic destroy_strategy,
    bool ALNS, double decay_factor, double reaction_factor,
    string init_algo_name, string replan_algo_name, bool sipp,
    int window_size_for_CT, int window_size_for_CAT, int window_size_for_PATH,
    int screen
): 
    instance(instance), path_table(path_table), agents(agents), HT(HT),
    init_algo_name(init_algo_name), replan_algo_name(replan_algo_name),
    window_size_for_CT(window_size_for_CT), window_size_for_CAT(window_size_for_CAT), window_size_for_PATH(window_size_for_PATH),
    screen(screen) {

    num_threads=omp_get_max_threads();

    // cout<<num_threads<<endl;
    // exit(-1);

    for (auto i=0;i<num_threads;++i) {
        auto local_optimizer=std::make_shared<LocalOptimizer>(
            instance, agents, HT,
            replan_algo_name, sipp,
            window_size_for_CT, window_size_for_CAT, window_size_for_PATH,
            screen
        );
        local_optimizers.push_back(local_optimizer);
    }

    neighbor_generator=std::make_shared<NeighborGenerator>(instance, path_table, agents, neighbor_size, destroy_strategy, ALNS, decay_factor, reaction_factor, num_threads, screen);
}

void GlobalManager::update(Neighbor & neighbor, bool recheck) {

    if (neighbor.succ){
        // before we update, we check if the solution is valid. because in the parallel setting, we may have later update that makes the solution invalid.

        if (recheck) {
            g_timer.record_p("manager_update_s");
        } else {
            g_timer.record_p("init_manager_update_s");
        }

        if (recheck) {
            g_timer.record_p("recheck_s");
            // re-check validness
            bool valid=true;
            for (auto & aid: neighbor.agents) {
                auto & path=neighbor.m_paths[aid];
                for (int i=0;i<path.size()-1;++i) {
                    int from=path[i].location;
                    int to=path[i+1].location;
                    int to_time=i+1;
                    // TODO(rivers): we need to ignore the conflicts with agents in the neighbor.
                    if (path_table.constrained(from,to,to_time,neighbor.agents)) {
                        std::cerr<<aid<<" "<<i<<" invalid"<<std::endl;
                        valid=false;
                        break;
                    }
                }
                if (!valid) break;
            }

            if (!valid) {
                neighbor.succ=false;
                std::cerr<<"invalid"<<std::endl;
                return;
            }

            // re-check if the cost is still smaller
            int old_sum_of_costs=0;
            for (auto & aid: neighbor.agents) {
                old_sum_of_costs+=agents[aid].path.size()-1;
                if (agents[aid].path.back().location!=instance.goal_locations[aid]) {
                    old_sum_of_costs+=HT->get(agents[aid].path.back().location,instance.goal_locations[aid]);
                }
            }

            if (old_sum_of_costs<neighbor.sum_of_costs) {
                neighbor.succ=false;
                std::cerr<<"incost"<<std::endl;
                return;
            } 

            // re-update old_sum_of_costs here.
            neighbor.old_sum_of_costs=old_sum_of_costs;
            for (auto & id: neighbor.agents) {
                neighbor.m_old_paths[id]=agents[id].path;
            }

            g_timer.record_d("recheck_s","recheck");

        }

        update(neighbor);

        if (recheck) {
            g_timer.record_d("manager_update_s","manager_update");
        } else {
            g_timer.record_d("init_manager_update_s","init_manager_update");
        }


        // synchonize to local optimizer
        if (recheck) {
            g_timer.record_p("loc_opt_update_s");
        } else {
            g_timer.record_p("init_loc_opt_update_s");
        }
        #pragma omp parallel for
        for (int i=0;i<num_threads;++i) {
            local_optimizers[i]->update(neighbor);
        }
        if (recheck) {
            g_timer.record_d("loc_opt_update_s","loc_opt_update");
        } else {
            g_timer.record_d("init_loc_opt_update_s","init_loc_opt_update");
        }

    }
}

void GlobalManager::update(Neighbor & neighbor) {
    // apply update
    g_timer.record_p("path_table_delete_s");
    for (auto & aid: neighbor.agents) {
        bool verbose=false;
        // if (neighbor.agents.size()<10){
        //     verbose=true;
        // } 
        path_table.deletePath(aid, neighbor.m_old_paths[aid],verbose);
    }
    g_timer.record_d("path_table_delete_s","path_table_delete");

    // std::cerr<<std::endl;
    g_timer.record_p("path_table_insert_s");
    for (auto & aid: neighbor.agents) {
        // update agents' paths here
        agents[aid].path = neighbor.m_paths[aid];
        bool verbose=false;
        // if (neighbor.agents.size()<10){
        //     verbose=true;
        // } 
        // update path table here
        path_table.insertPath(aid, neighbor.m_paths[aid],verbose);
    }
    g_timer.record_d("path_table_insert_s","path_table_insert");

        // std::cerr<<std::endl;
    // update costs here
    sum_of_costs += neighbor.sum_of_costs - neighbor.old_sum_of_costs;
}



// TODO(rivers): we will do single-thread code refactor first, then we will do the parallelization
bool GlobalManager::run(double time_limit) {

    TimeLimiter time_limiter(time_limit);

    initial_sum_of_costs=0;
    sum_of_costs=0;
    num_of_failures=0;
    average_group_size=0;
    iteration_stats.clear();

    double elapse=0;

    g_timer.record_p("lns_init_sol_s");
    sum_of_distances = 0;
    for (const auto & agent : agents)
    {
        sum_of_distances += HT->get(instance.start_locations[agent.id],instance.goal_locations[agent.id]);
    }

    // 0. get initial solution
    Neighbor init_neighbor;
    init_neighbor.agents.resize(agents.size());
    for (int i=0;i<agents.size();++i) {
        init_neighbor.agents[i]=i;
    }
    getInitialSolution(init_neighbor);

    update(init_neighbor,false);

    bool runtime=g_timer.record_d("lns_init_sol_s","lns_init_sol");

    elapse=time_limiter.get_elapse();
    iteration_stats.emplace_back(agents.size(), initial_sum_of_costs, runtime, init_algo_name);

    if (screen >= 1)
        cout << getSolverName() << " (only init executed): "
        << "runtime = " << runtime << ", "
        << "iterations = " << iteration_stats.size() << ", "
        << "solution cost = " << sum_of_costs << ", "
        << "initial solution cost = " << initial_sum_of_costs << ", "
        << "failed iterations = " << num_of_failures << endl;

    if (!init_neighbor.succ) {
        cerr << "Failed to get initial solution." << endl;
        exit(-1);
        return false;
    }

    g_timer.record_p("lns_opt_s");
    while (true) {
        if (time_limiter.timeout())
            break;

        // 1. generate neighbors
        g_timer.record_p("neighbor_generate_s");
        neighbor_generator->generate_parallel(time_limiter);
        g_timer.record_d("neighbor_generate_s","neighbor_generate");

        // 2. optimize the neighbor
        // auto neighbor_ptr = neighbor_generator.neighbors.front();
        // auto & neighbor = *neighbor_ptr;
        // neighbor_generator.neighbors.pop();

        // in the single-thread setting, local_optimizer directly modify paths
        // but we should first recover the path table, then redo it after all local optimizers finishes.
        g_timer.record_p("loc_opt_s");
        #pragma omp parallel for
        for (auto i=0;i<neighbor_generator->neighbors.size();++i) {
            // cerr<<i<<" "<<neighbor_generator.neighbors[i]->agents.size()<<endl;
            auto & neighbor_ptr = neighbor_generator->neighbors[i];
            auto & neighbor = *neighbor_ptr;
            local_optimizers[i]->optimize(neighbor, time_limiter);
        }
        g_timer.record_d("loc_opt_s","loc_opt");

        // TODO(rivers): validate solution

        // 3. update path_table, statistics & maybe adjust strategies
        g_timer.record_p("neighbor_update_s");
        // TODO(rivers): it seems we should not modify neighbor in the previous update
        for (auto & neighbor_ptr: neighbor_generator->neighbors){
            auto & neighbor=*neighbor_ptr;
            neighbor_generator->update(neighbor);
        }
        g_timer.record_d("neighbor_update_s","neighbor_update");

        for (auto & neighbor_ptr: neighbor_generator->neighbors){
            auto & neighbor=*neighbor_ptr;
            update(neighbor,true);

            if (!neighbor.succ) {
                ++num_of_failures;
            } 

            elapse=time_limiter.get_elapse();
            if (screen >= 1)
                cout << "Iteration " << iteration_stats.size() << ", "
                    << "group size = " << neighbor.agents.size() << ", "
                    << "solution cost = " << sum_of_costs << ", "
                    << "remaining time = " << time_limiter.time_limit-elapse << endl;
            iteration_stats.emplace_back(neighbor.agents.size(), sum_of_costs, elapse, replan_algo_name);
        }
    }
    g_timer.record_d("lns_opt_s","lns_opt");

    // TODO(rivers): validate solution

    average_group_size = - iteration_stats.front().num_of_agents;
    for (const auto& data : iteration_stats)
        average_group_size += data.num_of_agents;
    if (average_group_size > 0)
        average_group_size /= (double)(iteration_stats.size() - 1);

    elapse=time_limiter.get_elapse();
    cout << getSolverName() << ": "
        << "runtime = " << elapse << ", "
        << "iterations = " << iteration_stats.size() << ", "
        << "solution cost = " << sum_of_costs << ", "
        << "initial solution cost = " << initial_sum_of_costs << ", "
        << "failed iterations = " << num_of_failures << endl;

    return true;
}

void GlobalManager::getInitialSolution(Neighbor & neighbor) {
    // currently, we only support initial solution directly passed in.
    neighbor.old_sum_of_costs=0;
    neighbor.sum_of_costs=0;
    for (int i=0;i<agents.size();++i) {
        // cerr<<agents[i].id<<" "<< agents[i].path.size()-1<<endl;
        if (agents[i].path.back().location!=instance.goal_locations[i] && agents[i].path.size()<=window_size_for_CT) {
            cerr<<"A precomputed agent "<<i<<"'s path "<<agents[i].path.size()
                <<" should be longer than window size for CT "<<window_size_for_CT
                <<" unless it arrives at its goal:"<<agents[i].path.back().location
                <<" vs "<<instance.goal_locations[i]<<endl;
            exit(-1);
        }

        if (agents[i].path.back().location!=instance.goal_locations[i] && agents[i].path.size()!=window_size_for_PATH+1) {
            cerr<<"we require agent either arrives at its goal earlier or has a planned path of length window_size_for_PATH. "<<agents[i].path.size()<<" vs "<<window_size_for_PATH<<endl;
            exit(-1);
        }

        neighbor.sum_of_costs+=agents[i].path.size()-1;
        if (agents[i].path.back().location!=instance.goal_locations[i]) {
            neighbor.sum_of_costs+=HT->get(agents[i].path.back().location,instance.goal_locations[i]);
        }

        neighbor.m_paths[i]=agents[i].path;
        neighbor.m_old_paths[i]=Path();
        // cerr<<agents[i].id<<" "<< agents[i].path.size()-1<<endl;
    }

    neighbor.succ=true;

    initial_sum_of_costs = neighbor.sum_of_costs;
}


}