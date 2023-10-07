#include "LNS/Parallel/GlobalManager.h"
#include "util/Timer.h"

namespace LNS {

GlobalManager::GlobalManager(
    Instance & instance, PathTable & path_table, std::vector<Agent> & agents, std::shared_ptr<HeuristicTable> HT,
    int neighbor_size, destroy_heuristic destroy_strategy,
    bool ALNS, double decay_factor, double reaction_factor,
    string init_algo_name, string replan_algo_name, bool sipp,
    int window_size_for_CT, int window_size_for_CAT, int window_size_for_PATH,
    double time_limit, int screen
): 
    neighbor_generator(instance, path_table, agents, neighbor_size, destroy_strategy, ALNS, decay_factor, reaction_factor, screen),
    local_optimizer(instance, agents, HT, replan_algo_name, sipp, window_size_for_CT, window_size_for_CAT, window_size_for_PATH, screen),
    instance(instance), path_table(path_table), agents(agents), HT(HT),
    init_algo_name(init_algo_name), replan_algo_name(replan_algo_name),
    window_size_for_CT(window_size_for_CT), window_size_for_CAT(window_size_for_CAT), window_size_for_PATH(window_size_for_PATH),
    time_limit(time_limit), screen(screen) {

}

void GlobalManager::update(Neighbor & neighbor) {
    if (neighbor.succ){
        // before we update, we check if the solution is valid. because in the parallel setting, we may have later update that makes the solution invalid.

        


        for (auto & aid: neighbor.agents) {
            bool verbose=false;
            // if (neighbor.agents.size()<10){
            //     verbose=true;
            // } 

            path_table.deletePath(aid, neighbor.m_old_paths[aid],verbose);
        }

        // std::cerr<<std::endl;

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

        // std::cerr<<std::endl;
        // update costs here
        sum_of_costs += neighbor.sum_of_costs - neighbor.old_sum_of_costs;


        local_optimizer.update(neighbor);
    }
}

// TODO(rivers): we will do single-thread code refactor first, then we will do the parallelization
bool GlobalManager::run() {

    initial_sum_of_costs=0;
    sum_of_costs=0;
    num_of_failures=0;
    average_group_size=0;
    iteration_stats.clear();

    // we need an extra timer for this problem.
    g_timer.record_p("_lns_s");
    double elapse=0;

    sum_of_distances = 0;
    for (const auto & agent : agents)
    {
        sum_of_distances += HT->get(instance.start_locations[agent.id],instance.goal_locations[agent.id]);
    }

    // 0. get initial solution

    g_timer.record_p("lns_init_sol_s");
    Neighbor init_neighbor;
    init_neighbor.agents.resize(agents.size());
    for (int i=0;i<agents.size();++i) {
        init_neighbor.agents[i]=i;
    }
    getInitialSolution(init_neighbor);

    update(init_neighbor);

    bool runtime=g_timer.record_d("lns_init_sol_s","lns_init_sol");

    elapse=g_timer.record_d("_lns_s","_lns");
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

    while (true) {
        elapse=g_timer.record_d("_lns_s","_lns");
        if (elapse>=time_limit)
            break;

        // 1. generate neighbors
        if (neighbor_generator.neighbors.empty()){
            neighbor_generator.generate(1, time_limit);
        } 

        // 2. optimize the neighbor
        auto neighbor_ptr = neighbor_generator.neighbors.front();
        auto & neighbor = *neighbor_ptr;
        neighbor_generator.neighbors.pop();

        // in the single-thread setting, local_optimizer directly modify paths
        // but we should first recover the path table, then redo it after all local optimizers finishes.
        local_optimizer.optimize(neighbor, time_limit);

        // TODO(rivers): validate solution

        // 3. update path_table, statistics & maybe adjust strategies
        update(neighbor);
        neighbor_generator.update(neighbor);

        if (!neighbor.succ) {
            ++num_of_failures;
        } 

        sum_of_costs += neighbor.sum_of_costs - neighbor.old_sum_of_costs;

        elapse=g_timer.record_d("_lns_s","_lns");
        if (screen >= 1)
            cout << "Iteration " << iteration_stats.size() << ", "
                << "group size = " << neighbor.agents.size() << ", "
                << "solution cost = " << sum_of_costs << ", "
                << "remaining time = " << time_limit - elapse << endl;
        iteration_stats.emplace_back(neighbor.agents.size(), sum_of_costs, elapse, replan_algo_name);
    }

    // TODO(rivers): validate solution

    average_group_size = - iteration_stats.front().num_of_agents;
    for (const auto& data : iteration_stats)
        average_group_size += data.num_of_agents;
    if (average_group_size > 0)
        average_group_size /= (double)(iteration_stats.size() - 1);

    elapse=g_timer.record_d("_lns_s","_lns");
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
        // cerr<<agents[i].id<<" "<< agents[i].path.size()-1<<endl;
    }

    neighbor.succ=true;

    initial_sum_of_costs = neighbor.sum_of_costs;
}


}