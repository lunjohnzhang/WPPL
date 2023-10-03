#pragma once
#include "SharedEnv.h"
#include <omp.h>
#include "LaCAM2/SUO/Search/SpatialSearch.h"
#include "util/HeuristicTable.h"


namespace SUO {


class SUO {
public:
    SUO(
        const SharedEnvironment & _env,
        int _n_orients,
        const std::vector<int> & _weights,
        std::shared_ptr<HeuristicTable> & _HT,
        float _vertex_collision_cost,
        int _iterations,
        int _max_expanded,
        float _h_weight
        ):
        env(_env),
        weights(_weights),
        n_orients(_n_orients),
        HT(_HT),
        vertex_collision_cost(_vertex_collision_cost), 
        iterations(_iterations),
        max_expanded(_max_expanded),
        h_weight(_h_weight) {
        
        n_threads=omp_get_max_threads();

        planners = new SpatialAStar * [n_threads];
        for (int tid=0;tid<n_threads;++tid) {
            planners[tid] = new SpatialAStar(env, n_orients, weights);
        }
    }


    ~SUO() {
        for (int tid=0;tid<n_threads;++tid) {
            delete planners[tid];
        }
        delete [] planners;
    }

    int n_threads;
    SpatialAStar ** planners;

    int n_orients;
    const SharedEnvironment & env;
    const std::vector<int> & weights;
    std::shared_ptr<HeuristicTable> HT;

    float vertex_collision_cost;
    int iterations;
    int max_expanded;
    float h_weight;

    std::vector<std::vector<State> > paths;
    std::vector<float> path_costs;
    std::vector<int> orders;
    std::vector<float> cost_map;
    
    void init();
    void plan();
    void update_path(int agent_idx, State * goal_state, bool update_cost_map=true);
    void reset_cost_map();

};

} // namespace SUO