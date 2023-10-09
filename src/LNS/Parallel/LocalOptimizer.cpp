#include "LNS/Parallel/LocalOptimizer.h"
#include "util/Timer.h"
#include "util/Dev.h"

namespace LNS {

LocalOptimizer::LocalOptimizer(
    Instance & instance, std::vector<Agent> & agents, std::shared_ptr<HeuristicTable> HT,
    string replan_algo_name, bool sipp,
    int window_size_for_CT, int window_size_for_CAT, int window_size_for_PATH,
    int screen
):
    instance(instance), path_table(instance.map_size,window_size_for_CT), agents(agents), HT(HT),
    replan_algo_name(replan_algo_name),
    window_size_for_CT(window_size_for_CT), window_size_for_CAT(window_size_for_CAT), window_size_for_PATH(window_size_for_PATH),
    screen(screen) {

    // TODO(rivers): for agent_id, we just use 0 to initialize the path planner. but we need to change it (also starts and goals) everytime before planning
    if(sipp)
        path_planner = std::make_shared<SIPP>(instance, 0, HT);
    else
        path_planner = std::make_shared<SpaceTimeAStar>(instance, 0, HT);

}

void LocalOptimizer::update(Neighbor & neighbor) {
    if (neighbor.succ) {
        for (auto & aid: neighbor.agents) {
            path_table.deletePath(aid, neighbor.m_old_paths[aid]);
        }

        for (auto & aid: neighbor.agents) {
            path_table.insertPath(aid, neighbor.m_paths[aid]);
        }
    }
}

void LocalOptimizer::prepare(Neighbor & neighbor) {
    // store the neighbor information
    //ONLYDEV(g_timer.record_p("store_neighbor_info_s");)
    neighbor.old_sum_of_costs = 0;
    for (auto & aid: neighbor.agents)
    {
        auto & agent=agents[aid];
        if (replan_algo_name == "PP")
            neighbor.m_old_paths[aid] = agent.path;
        // path_table.deletePath(neighbor.agents[i], agent.path);
        neighbor.old_sum_of_costs += agent.path.size() - 1;
        if (agent.path.back().location!=instance.goal_locations[aid]) {
            neighbor.old_sum_of_costs+=HT->get(agent.path.back().location,instance.goal_locations[aid]);
        }

        path_table.deletePath(aid, neighbor.m_old_paths[aid]);
    }   

    //ONLYDEV(g_timer.record_d("store_neighbor_info_s","store_neighbor_info_e","store_neighbor_info");)    
}

void LocalOptimizer::optimize(Neighbor & neighbor, const TimeLimiter & time_limiter) {

    prepare(neighbor);

    // replan
    //ONLYDEV(g_timer.record_p("replan_s");)
    bool succ=false;
    if (replan_algo_name == "PP")
        succ = runPP(neighbor, time_limiter);
    else
    {
        cerr << "Wrong replanning strategy" << endl;
        exit(-1);
    }
    if (!succ) {
        if (screen>=1)
            g_logger.debug("replan failed");
    }
    //ONLYDEV(g_timer.record_d("replan_s","replan_e","replan");)

    // the cleanup is done in runPP: e.g., if runPP fails, the old paths are restored.

    neighbor.succ=succ;
}

bool LocalOptimizer::runPP(Neighbor & neighbor, const TimeLimiter & time_limiter)
{
    //ONLYDEV(g_timer.record_p("run_pp_s");)
    auto shuffled_agents = neighbor.agents;
    std::random_shuffle(shuffled_agents.begin(), shuffled_agents.end());
    if (screen >= 2) {
        for (auto aid : shuffled_agents)
            cout << aid << "(" << HT->get(neighbor.m_paths[aid].back().location,instance.goal_locations[aid]) << "->" << agents[aid].path.size() - 1 << "), ";
        cout << endl;
    }
    int remaining_agents = (int)shuffled_agents.size();
    auto p = shuffled_agents.begin();
    neighbor.sum_of_costs = 0;
    CBSNode node;
    int suboptimality=1.2;
    int search_priority=1;
    bool use_soft_constraint=true;
    ConstraintTable constraint_table(instance.num_of_cols, instance.map_size, &path_table, nullptr, window_size_for_CT, window_size_for_CAT, window_size_for_PATH);

    // TODO(rivers): we require the path to be at least window_size_for_PATH
    // TODO(rivers): we use hold goal location assumption here, which is not necessary.
    // if (window_size_for_PATH!=MAX_TIMESTEP) {
    //     constraint_table.length_min=window_size_for_PATH;
    // }

    while (p != shuffled_agents.end()) {
        if (time_limiter.timeout())
            break;

        int id = *p;
        if (screen >= 3)
            cout << "Remaining agents = " << remaining_agents <<
                 ", remaining time = " << time_limiter.get_remaining_time() << " seconds. " << endl
                 << "Agent " << agents[id].id << endl;
        if (search_priority==1) {
            //ONLYDEV(g_timer.record_p("findPath_s");)
            path_planner->prepare_for_planning(id);
            neighbor.m_paths[id] = path_planner->findPath(constraint_table);
            //ONLYDEV(g_timer.record_d("findPath_s","findPath_e","findPath");)
        } else if (search_priority==2) {
            std::cerr<<"not supported now, need double checks"<<std::endl;
            exit(-1);
            vector<Path *> paths(agents.size(),nullptr);
            int min_f_val;
            tie(neighbor.m_paths[id],min_f_val) = path_planner->findSuboptimalPath(node, constraint_table, paths, agents[id].id, 0,suboptimality);
        }
        if (neighbor.m_paths[id].empty()) break;
        
        // always makes a fixed length.
        // TODO(rivers): this might not be a smart choice, but it keeps everything consistent.
        if (neighbor.m_paths[id].size()>constraint_table.window_size_for_PATH+1) {
            neighbor.m_paths[id].resize(constraint_table.window_size_for_PATH+1);
        }

        // do we need to pad here?
        // assume hold goal location
        if (neighbor.m_paths[id].size()<constraint_table.window_size_for_PATH+1) {
            neighbor.m_paths[id].resize(constraint_table.window_size_for_PATH+1,neighbor.m_paths[id].back());
        }

        neighbor.sum_of_costs += (int)neighbor.m_paths[id].size() - 1;
        // if (neighbor.m_paths[id].back().location!=agents[id].path_planner->goal_location) {
            if (neighbor.m_paths[id].size()!=constraint_table.window_size_for_PATH+1) {
                std::cerr<<"agent "<<agents[id].id<<"'s path length "<<neighbor.m_paths[id].size()<<" should be equal to window size for path "<<constraint_table.window_size_for_PATH<< "if it doesn't arrive at its goal"<<endl;
                exit(-1);
            } else {
                neighbor.sum_of_costs+=HT->get(neighbor.m_paths[id].back().location, path_planner->goal_location);
            }
        // }
        if (neighbor.sum_of_costs >= neighbor.old_sum_of_costs){
            // because it is not inserted into path table yet.
            neighbor.m_paths[id].clear();
            break;
        }
        remaining_agents--;
        path_table.insertPath(agents[id].id, neighbor.m_paths[id]);
        ++p;
    }

    // remove any insertion from new paths
    for (auto & aid: neighbor.agents) {
        path_table.deletePath(aid, neighbor.m_paths[aid]);
    }

    // restore old paths
    if (!neighbor.m_old_paths.empty())
    {
        for (auto & aid : neighbor.agents) {
            path_table.insertPath(aid, neighbor.m_old_paths[aid]);
        }
    }
    //ONLYDEV(g_timer.record_d("run_pp_s","run_pp_e","run_pp");)


    if (remaining_agents == 0 && neighbor.sum_of_costs <= neighbor.old_sum_of_costs) // accept new paths
    {
        return true;
    }
    else 
    {   
        return false;
    }
}





}