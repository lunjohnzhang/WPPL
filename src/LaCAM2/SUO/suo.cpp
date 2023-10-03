#include "LaCAM2/SUO/suo.hpp"
#include <algorithm>

namespace SUO {

void SUO::init() {
    paths.clear();
    paths.resize(env.num_of_agents);
    path_costs.clear();
    path_costs.resize(env.num_of_agents, FLT_MAX);
    orders.resize(env.num_of_agents);
    std::iota(orders.begin(),orders.end(), 0);
    reset_cost_map();
}

void SUO::reset_cost_map() {
    cost_map.clear();
    cost_map.resize(env.map.size(), 0);
}

void SUO::plan() {
    init();

    std::vector<int> distance(env.num_of_agents, 0);
    for (int i=0;i<env.num_of_agents;++i) {
        distance[i]=HT->get(env.curr_states[i].location, env.goal_locations[i][0].first);
    }

    // we need to sort agents first
    std::sort(orders.begin(), orders.end(), [&](int i, int j) {
        return distance[i]<distance[j];
    });

    // TODO(rivers): we should add their shortest path as initialization?


    // TODO(rivers): how to update cost map asynchronously? maybe everyone needs to hold its own copy of cost map
    // and before planning, it checks whether should update the cost map first. we could keep delta cost in a vector,
    // and each thread has a pointer to where it has been updated to.

    // we need to keep cost map for each thread anyway, because before planning for each agent, we need to remove its own old path costs from the cost map.

    // then for each agent, we plan with Spatial A* search
    g_logger.debug("SUO: plan with {} iterations, {} threads for {} agents", iterations, n_threads, env.num_of_agents);
    for (int j=0;j<iterations;++j){
        // TODO(rivers): currently we use a batch mode
        int num_batches=(env.num_of_agents+n_threads-1)/n_threads;
        for (int bid=0;bid<num_batches;++bid){
            int start_idx=bid*n_threads;
            int end_idx=min((bid+1)*n_threads, env.num_of_agents);

            std::vector<std::pair<int,State *>> goal_states;
            // TODO(rivers): we need to make sure omp actually uses the number of threads we want
            #pragma omp parallel for schedule(static,1)
            for (int i=start_idx;i<end_idx;++i) {

                int tid=omp_get_thread_num();
                auto planner = planners[tid];
                int agent_idx = orders[i];
                int start_pos = env.curr_states[agent_idx].location;
                int goal_pos = env.goal_locations[agent_idx][0].first;
                
                auto old_path=paths[agent_idx];
                
                // update cost map first
                // if (j!=0)
                planner->update_cost_map(cost_map, old_path, vertex_collision_cost);
                planner->reset_plan();

                State * goal_state = planner->search(start_pos, -1, goal_pos);
                if (goal_state==nullptr) {
                    cerr<<"SUO: agent "<<agent_idx<<" failed to find a path"<<endl;
                    exit(-1);
                }

                
                #pragma omp critical 
                {
                    goal_states.emplace_back(agent_idx, goal_state);
                }
        

                // g_timer.record_p("Astar_s");
                // double d=g_timer.record_d("Astar_s","Astar");
                // cerr<<"a star time cost"<<d<<endl;
            }

            for (int i=0;i<goal_states.size();++i) {
                auto & p = goal_states[i];
                update_path(p.first, p.second);
            }
        }
    }
}

void SUO::update_path(int agent_idx, State * goal_state, bool update_cost_map) {
    // NOTE(rivers) we cannot compare f to decided whether to replace the old path.
    // float old_f = path_costs[agent_idx];
    // // TODO(rives): we should prefer less conflicts as well
    // if (goal_state->f>old_f) {
    //     return;
    // }

    auto & path=paths[agent_idx]; 

    if (update_cost_map) {
        
        // remove old path from the cost map
        for (auto &state: path) {
            cost_map[state.pos]-=vertex_collision_cost;
        }
    }

    path_costs[agent_idx]=goal_state->f;
    path.clear();

    State * s=goal_state;
    path.push_back(State(s->pos));
    while (s->prev!=nullptr) {
        s=s->prev;
        path.push_back(State(s->pos));
    }

    std::reverse(path.begin(), path.end());

    if (update_cost_map) {
        // add new path to the cost map
        for (auto &state: path) {
            cost_map[state.pos]+=vertex_collision_cost;
        }
    }
}

}