#pragma once
#include "common.h"
#include "LNS/Instance.h"

namespace LNS {

namespace Parallel {

struct PathEntry {
    int location;
    int orientation;
    PathEntry(int location=-1, int orientation=-1): location(location), orientation(orientation) {}
};

struct Path {
    std::vector<PathEntry> nodes; // pos, orient
    float path_cost;
    inline void clear() {
        path_cost=0;
        nodes.clear();
    }

    inline PathEntry & operator [] (int i) { return nodes[i];}
    inline size_t size() {return nodes.size();}
    inline bool empty() {return nodes.empty();}

    inline PathEntry & back() {return nodes.back();}
    inline PathEntry & front() {return nodes.front();}

};

struct Agent
{
    int id;
    Path path;
    const Instance & instance;
    std::shared_ptr<HeuristicTable> HT; // instance

    Agent(int id, const Instance& instance, std::shared_ptr<HeuristicTable> & HT): id(id), HT(HT), instance(instance) {}

    inline int getStartLocation() {return instance.start_locations[id];}
    inline int getStartOrientation() {return instance.start_orientations[id];}
    inline int getGoalLocation() {return instance.goal_locations[id];}

    inline int getEstimatedPathLength() {
        return (int) path.size() -1 + HT->get(path.back().location, path.back().orientation, instance.goal_locations[id]);
    }

    inline int getNumOfDelays() {
        // TODO(rivers): we may need two heuristic table: one for cost, one for path length estimation.
        return getEstimatedPathLength() - HT->get(instance.start_locations[id],instance.start_orientations[id],instance.goal_locations[id]);
    }
};

struct Neighbor
{
    vector<int> agents;
    int sum_of_costs;
    int old_sum_of_costs;
    std::map<int, Path> m_paths; // for temporally storing the new paths. may change to vector later, agent id -> path
    // set<pair<int, int>> colliding_pairs;  // id1 < id2
    // set<pair<int, int>> old_colliding_pairs;  // id1 < id2
    // std::vector<Path> old_paths;
    std::map<int, Path> m_old_paths; // for temporally storing the old paths. may change to vector later, agent id -> path
    bool succ = false;
    int selected_neighbor;
};

}

} // namespace LNS