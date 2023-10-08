#include "LNS/Parallel/NeighborGenerator.h"
#include "util/Timer.h"
#include "util/Dev.h"
#include "util/MyLogger.h"
#include <algorithm>
#include "omp.h"

namespace LNS {

NeighborGenerator::NeighborGenerator(
    Instance & instance, PathTable & path_table, std::vector<Agent> & agents, 
    int neighbor_size, destroy_heuristic destroy_strategy, 
    bool ALNS, double decay_factor, double reaction_factor, 
    int num_threads, int screen
):
    instance(instance), path_table(path_table), agents(agents),
    neighbor_size(neighbor_size), destroy_strategy(destroy_strategy),
    ALNS(ALNS), decay_factor(decay_factor), reaction_factor(reaction_factor),
    num_threads(num_threads), screen(screen) {

    destroy_weights.assign(DESTORY_COUNT,1);

    // if (intersections.empty())
    // {
    for (int i = 0; i < instance.map_size; i++)
    {
        if (!instance.isObstacle(i) && instance.getDegree(i) > 2)
            intersections.push_back(i);
    }
    // }

    tabu_list_list.resize(num_threads);
    neighbors.resize(num_threads);

}

void NeighborGenerator::update(Neighbor & neighbor){
    if (ALNS) // update destroy heuristics
    {
        if (neighbor.old_sum_of_costs > neighbor.sum_of_costs )
            destroy_weights[neighbor.selected_neighbor] =
                    reaction_factor * (neighbor.old_sum_of_costs - neighbor.sum_of_costs) / neighbor.agents.size()
                    + (1 - reaction_factor) * destroy_weights[neighbor.selected_neighbor];
        else
            destroy_weights[neighbor.selected_neighbor] =
                    (1 - decay_factor) * destroy_weights[neighbor.selected_neighbor];
    }
}

void NeighborGenerator::generate_parallel(double time_limit) {
    #pragma omp parallel for
    for (int i = 0; i < num_threads; i++) {
        generate(time_limit,i);
    }
}

void NeighborGenerator::generate(double time_limit,int idx) {
    std::shared_ptr<Neighbor> neighbor_ptr = std::make_shared<Neighbor>();
    Neighbor & neighbor = *neighbor_ptr;

    bool succ=false;
    while (!succ){
        double elapse=g_timer.record_d("_lns_s","_lns");
        if (elapse>=time_limit)
            break;

        if (ALNS)
            chooseDestroyHeuristicbyALNS();

        ONLYDEV(g_timer.record_p("generate_neighbor_s");)
        switch (destroy_strategy)
        {
            case RANDOMWALK:
                {
                    succ = generateNeighborByRandomWalk(neighbor,idx);
                    neighbor.selected_neighbor = 0;
                    break;
                }
            case INTERSECTION:
                {
                    succ = generateNeighborByIntersection(neighbor);
                    neighbor.selected_neighbor = 1;
                    break;
                }
            case RANDOMAGENTS:
                // TODO(rivers): this implementation is too bad
                // neighbor.agents.resize(agents.size());
                // for (int i = 0; i < (int)agents.size(); i++)
                //     neighbor.agents[i] = i;
                // if (neighbor.agents.size() > neighbor_size)
                // {
                //     std::random_shuffle(neighbor.agents.begin(), neighbor.agents.end());
                //     neighbor.agents.resize(neighbor_size);
                // }
                // succ = true;
                // neighbor.selected_neighbor = 2;
                {
                    auto s=std::set<int>();
                    while (s.size()<neighbor_size) {
                        s.insert(rand()%agents.size());
                    }
                    for (auto i:s) {
                        neighbor.agents.push_back(i);
                    } 
                    succ = true;
                    neighbor.selected_neighbor = 2;
                    break;
                }
            default:
                cerr << "Wrong neighbor generation strategy" << endl;
                exit(-1);
        }
        ONLYDEV(g_timer.record_d("generate_neighbor_s","generate_neighbor_e","generate_neighbor");)

        if (!succ) {        
            // TODO: we need to count how many times we failed to generate a neighbor
            if (screen>=1)
                g_logger.debug("generate neighbors failed");
        }
    }

    neighbors[idx]=neighbor_ptr;
}

void NeighborGenerator::chooseDestroyHeuristicbyALNS() {
    int selected_neighbor=rouletteWheel();
    switch (selected_neighbor)
    {
        case 0 : destroy_strategy = RANDOMWALK; break;
        case 1 : destroy_strategy = INTERSECTION; break;
        case 2 : destroy_strategy = RANDOMAGENTS; break;
        default : cerr << "ERROR" << endl; exit(-1);
    }
}

int NeighborGenerator::rouletteWheel()
{
    double sum = 0;
    for (const auto& h : destroy_weights)
        sum += h;
    if (screen >= 2)
    {
        cout << "destroy weights = ";
        for (const auto& h : destroy_weights)
            cout << h / sum << ",";
        cout << endl;
    }
    double r = (double) rand() / RAND_MAX;
    double threshold = destroy_weights[0];
    int selected_neighbor = 0;
    while (threshold < r * sum)
    {
        selected_neighbor++;
        threshold += destroy_weights[selected_neighbor];
    }
    return selected_neighbor;
}

bool NeighborGenerator::generateNeighborByRandomWalk(Neighbor & neighbor, int idx) {
    if (neighbor_size >= (int)agents.size())
    {
        neighbor.agents.resize(agents.size());
        for (int i = 0; i < (int)agents.size(); i++)
            neighbor.agents[i] = i;
        return true;
    }

    int a = findMostDelayedAgent(idx);
    if (a < 0)
        return false;
    
    set<int> neighbors_set;
    neighbors_set.insert(a);
    randomWalk(a, agents[a].path[0].location, 0, neighbors_set, neighbor_size, (int) agents[a].path.size() - 1);

    // TODO(rivers): we iterate for at most 10 iterations (not shown in the pseudo-code) to 
    // address the situation where the agent density is too low for us to collect N agents
    int count = 0;
    while (neighbors_set.size() < neighbor_size && count < 10) {
        int t = rand() % agents[a].path.size();
        randomWalk(a, agents[a].path[t].location, t, neighbors_set, neighbor_size, (int) agents[a].path.size() - 1);
        count++;
        // select the next agent randomly
        int idx = rand() % neighbors_set.size();
        int i = 0;
        for (auto n : neighbors_set)
        {
            if (i == idx)
            {
                a = i;
                break;
            }
            i++;
        }
    }
    if (neighbors_set.size() < 2)
        return false;

    neighbor.agents.assign(neighbors_set.begin(), neighbors_set.end());
    if (screen >= 2)
        cout << "Generate " << neighbor.agents.size() << " neighbors by random walks of agent " << a
             << "(" << agents[a].path_planner->HT->get(agents[a].path_planner->start_location,agents[a].path_planner->goal_location)
             << "->" << agents[a].path.size() - 1 << ")" << endl;

    return true;
}

bool NeighborGenerator::generateNeighborByIntersection(Neighbor & neighbor) {
    set<int> neighbors_set;
    auto pt = intersections.begin();
    std::advance(pt, rand() % intersections.size());
    int location = *pt;
    path_table.get_agents(neighbors_set, neighbor_size, location);
    if (neighbors_set.size() < neighbor_size)
    {
        set<int> closed;
        closed.insert(location);
        std::queue<int> open;
        open.push(location);
        while (!open.empty() && (int) neighbors_set.size() < neighbor_size)
        {
            int curr = open.front();
            open.pop();
            for (auto next : instance.getNeighbors(curr))
            {
                if (closed.count(next) > 0)
                    continue;
                open.push(next);
                closed.insert(next);
                if (instance.getDegree(next) >= 3)
                {
                    path_table.get_agents(neighbors_set, neighbor_size, next);
                    if ((int) neighbors_set.size() == neighbor_size)
                        break;
                }
            }
        }
    }
    neighbor.agents.assign(neighbors_set.begin(), neighbors_set.end());
    if (neighbor.agents.size() > neighbor_size)
    {
        std::random_shuffle(neighbor.agents.begin(), neighbor.agents.end());
        neighbor.agents.resize(neighbor_size);
    }
    if (screen >= 2)
        cout << "Generate " << neighbor.agents.size() << " neighbors by intersection " << location << endl;
    return true;
}

int NeighborGenerator::findMostDelayedAgent(int idx){
    int a = -1;
    int max_delays = -1;
    auto & tabu_list=tabu_list_list[idx];
    for (int i = 0; i < agents.size(); i++)
    {
        // TODO(rivers): currently we just use index to split threads
        if (i%num_threads!=idx) continue;
        if (tabu_list.find(i) != tabu_list.end())
            continue;
        int delays = agents[i].getNumOfDelays();
        if (max_delays < delays)
        {
            a = i;
            max_delays = delays;
        }
    }
    if (max_delays == 0)
    {
        tabu_list.clear();
        return -1;
    }
    tabu_list.insert(a);
    if (tabu_list.size() == agents.size())
        tabu_list.clear();
    return a;
}


// a random walk with path that is shorter than upperbound and has conflicting with neighbor_size agents
void NeighborGenerator::randomWalk(int agent_id, int start_location, int start_timestep,
                     set<int>& conflicting_agents, int neighbor_size, int upperbound)
{
    int loc = start_location;
    for (int t = start_timestep; t < upperbound; t++)
    {
        auto next_locs = instance.getNeighbors(loc);
        next_locs.push_back(loc);
        while (!next_locs.empty())
        {
            int step = rand() % next_locs.size();
            auto it = next_locs.begin();
            advance(it, step);
            int next_h_val = agents[agent_id].path_planner->HT->get(*it,agents[agent_id].path_planner->goal_location);
            if (t + 1 + next_h_val < upperbound) // move to this location
            {
                path_table.getConflictingAgents(agent_id, conflicting_agents, loc, *it, t + 1);
                loc = *it;
                break;
            }
            next_locs.erase(it);
        }
        if (next_locs.empty() || conflicting_agents.size() >= neighbor_size)
            break;
    }
}



}