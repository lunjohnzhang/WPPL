#include "LaCAM2/SUO/suo_st.hpp"

namespace LaCAM2 {

void SUO::init(Instance & instance){
    paths.clear();
    paths.resize(instance.N);
    path_costs.clear();
    path_costs.resize(instance.N, FLT_MAX);
    cost_map.clear();
    cost_map.resize(instance.G.width*instance.G.height, 0);
    orders.resize(instance.N);
    std::iota(orders.begin(), orders.end(), 0);
}

void SUO::plan(Instance & instance){
    init(instance);

    // TODO: In refining loop

    // we need to sort agents first
    std::sort(orders.begin(), orders.end(), [&](int i, int j) {
        const AgentInfo & a=instance.agent_infos[i];
        const AgentInfo & b=instance.agent_infos[j];

        int h1=HT->get(instance.starts[i]->index,instance.goals[i]->index);
        int h2=HT->get(instance.starts[j]->index,instance.goals[j]->index);

        if (h1!=h2) return h1<h2;

        if (a.elapsed!=b.elapsed) return a.elapsed>b.elapsed;

        return a.tie_breaker>b.tie_breaker;
    });

    // then for each agent, we plan with Spatial A* search
    for (int j=0;j<iterations;++j){
        for (int i=0;i<instance.N;i++) {
            int agent_idx=orders[i];
            // g_timer.record_p("Astar_s");
            AStar(agent_idx, instance.starts[agent_idx], instance.goals[agent_idx]);
            // double d=g_timer.record_d("Astar_s","Astar");
            // cerr<<"a star time cost"<<d<<endl;
        }
    }
    
}

void SUO::AStar(int agent_idx, Vertex * start, Vertex * goal) {
    open_list.clear();
    for (auto & s : all_states) {
        delete s;
    }
    all_states.clear();

    auto start_state = new State(start, 0, HT->get(start->index, goal->index), nullptr);
    all_states.insert(start_state);
    start_state->closed = false;
    start_state->open_handle = open_list.push(start_state);


    int num_expanded=0;
    while (!open_list.empty()) {
        auto curr_state = open_list.top();
        open_list.pop();
        curr_state->closed = true;
        ++num_expanded;

        if (max_expanded>=0 && num_expanded==max_expanded){
            update_path(agent_idx, curr_state);
            return;
        }

        if (curr_state->v->index==goal->index) {
            // we need to reconstruct the path
            update_path(agent_idx, curr_state);
            return;
        }

        // NOTE(hj): we use no rotation action models for now.
        for (auto neighbor_vertex: curr_state->v->neighbor) {
            // TODO(hj): we need to retrive the updated cost from the heatmap;
            int cost=cost_map[neighbor_vertex->index]+1;
            auto next_state= new State(neighbor_vertex, curr_state->g+cost, h_weight*HT->get(neighbor_vertex->index, goal->index), curr_state);
            if (next_state->v==0){
                cerr<<"next state is null"<<endl;
                exit(-1);
            }

            auto iter=all_states.find(next_state);
            if (iter==all_states.end()) {
                // new state
                all_states.insert(next_state);
                next_state->closed = false;
                next_state->open_handle = open_list.push(next_state);
            } else {
                // old state
                auto old_state = *iter;
                if (next_state->g<old_state->g) {
                    // we need to update the state
                    old_state->copy(next_state);
                    if (old_state->closed) {
                        old_state->closed = false;
                        old_state->open_handle = open_list.push(old_state);
                    } else {
                        open_list.increase(old_state->open_handle);
                    }
                }
                delete next_state;
            }
        }

    }

}

void SUO::update_path(int agent_idx, State * goal_state) {
    float old_f = path_costs[agent_idx];
    // TODO(rives): we should prefer less conflicts as well
    if (goal_state->f>old_f) {
        return;
    }

    // remove old path from the cost map
    auto & path=paths[agent_idx]; 
    for (auto &state: path) {
        cost_map[state.v->index]-=vertex_collision_cost;
    }

    path_costs[agent_idx]=goal_state->f;
    path.clear();

    State * s=goal_state;
    path.push_back(State(s->v));
    while (s->prev!=nullptr) {
        s=s->prev;
        path.push_back(State(s->v));
    }

    std::reverse(path.begin(), path.end());

    // add new path to the cost map
    for (auto &state: path) {
        cost_map[state.v->index]+=vertex_collision_cost;
    }
}

} // namespace LaCAM2