#pragma once

#include <memory>
#include <vector>
#include "Routing/Region.h"
#include "SharedEnv.h"
#include "Routing/AgentController.h"
#include "Routing/State.h"

namespace Routing {

struct GlobalController
{
    std::vector<std::shared_ptr<Region> > regions;
    std::vector<AgentController> agent_controllers;

    std::shared_ptr<HeuristicTable> HT;
    std::shared_ptr<vector<float> > weights;
    const SharedEnvironment & env;

    boost::heap::pairing_heap<State*, boost::heap::compare<State::Compare> > open_list;
    boost::unordered_set<State*, State::Hash, State::Equal> all_states;

    std::vector<State*> successors;


    int sy=8;
    int sx=8;

    int ny;
    int nx;

    GlobalController(const SharedEnvironment & env, std::shared_ptr<HeuristicTable> HT, std::shared_ptr<vector<float> > weights):
        env(env), HT(HT), weights(weights) {
        build_graph();
        for (int i=0;i<env.num_of_agents;++i) {
            agent_controllers.emplace_back(i, regions, HT, weights, env);
        }
    }

    void build_graph() {
        int height = env.rows;
        int width = env.cols;

        ny=(height+sy-1)/sy;
        nx=(width+sx-1)/sx;

        // build nodes
        for (int y=0;y<ny;++y){
            for (int x=0;x<nx;++x){
                int id=y*nx+x;
                auto r=std::make_shared<Region>(id,y,x);

                // TODO: we need to add boundary for each region
                for (int i=0;i<sy;++i){
                    for (int j=0;j<sx;++j){
                        if (i==0 || i==sy-1 || j==0 || j==sx-1) {
                            int _y=y*sy+i;
                            int _x=x*sx+j;
                            int _pos=_y*env.cols+_x;
                            if (env.map[_pos]==0) {
                                // this is a boundary cell
                                r->boundary.insert(_pos);
                            }
                        }
                    }
                }

                regions.push_back(r);
            }
        }


        // build edges
        for (int y=0;y<ny;++y){
            for (int x=0;x<nx;++x){
                int id=y*nx+x;
                auto r=regions[id];
                if (y>0){
                    r->neighbors.push_back(regions[(y-1)*nx+x]);
                }
                if (y<ny-1){
                    r->neighbors.push_back(regions[(y+1)*nx+x]);
                }
                if (x>0){
                    r->neighbors.push_back(regions[y*nx+x-1]);
                }
                if (x<nx-1){
                    r->neighbors.push_back(regions[y*nx+x+1]);
                }
            }
        }
    }


    int get_region_index(int pos) {
        int y=pos/env.cols;
        int x=pos%env.cols;

        int ry=y/sy;
        int rx=x/sx;

        int id=ry*nx+rx;
        return id;
    }

    void clear() {
        open_list.clear();
        for (auto s: all_states) {
            delete s;
        }
        all_states.clear();
    }


    void build_path(State * curr, int agent_id) {
        auto s=curr;
        agent_controllers[agent_id].region_path.push_back(s->pos);
        while (s->prev!=nullptr) {
            s=s->prev;
            agent_controllers[agent_id].region_path.push_back(s->pos);
        }
        std::reverse(agent_controllers[agent_id].region_path.begin(), agent_controllers[agent_id].region_path.end());

        // add congestion to the regions that are on the path
        for (auto & region_id: agent_controllers[agent_id].region_path) {
            regions[region_id]->congestion++;
        }

    }

    int get_h(int region, int goal_region) {
        int y=region/nx;
        int x=region%nx;

        int gy=goal_region/nx;
        int gx=goal_region%nx;

        return abs(gy-y)+abs(gx-x); // Manhattan distance
    }

    void getSuccessors(State * curr, int goal_region) {
        successors.clear();

        int region=curr->pos;

        for (auto & neighbor: regions[region]->neighbors) {
            int next_region=neighbor->id;
            int next_g=curr->g+1;
            int next_h=get_h(next_region, goal_region);
            int next_max_congestion=std::max(curr->max_congestion, neighbor->congestion);
            State * next_state=new State(next_region, -1, next_g, next_h, next_max_congestion, curr);
            successors.push_back(next_state);
        }

    }

    // we need to take agents' starts and goals as input
    // and output a region path for each agent
    void findPath(int agent_id, int start_pos, int goal_pos) {
        clear();
        
        if (agent_controllers[agent_id].region_path.size()!=0) {
            // agent is already on the path
            cout<<"agent "<<agent_id<<" already has a path\n";
            exit(-1);
        }

        int goal_region=get_region_index(goal_pos);
        int start_region=get_region_index(start_pos);
        int congestion=regions[start_region]->congestion;
        State *start=new State(start_pos, -1, 0, 0, congestion, nullptr);
        all_states.insert(start);
        start->closed=false;
        start->open_list_handle=open_list.push(start);

        while (!open_list.empty()) {
            State * curr=open_list.top();
            open_list.pop();
            curr->closed=true;

            if (curr->pos==goal_region) {
                // we find a path
                build_path(curr, agent_id);
                break;
            }

            getSuccessors(curr, goal_region);
            for (auto & next_state: successors) {
                auto iter = all_states.find(next_state);
                if (iter==all_states.end()) {
                    // new state
                    all_states.insert(next_state);
                    next_state->closed=false;
                    next_state->open_list_handle=open_list.push(next_state);
                } else {
                    // old state
                    auto old_state=(*iter);
                    if (
                        next_state->f<old_state->f || (
                            next_state->f==old_state->f
                            && next_state->max_congestion<old_state->max_congestion
                        )
                    ) {
                        // we need to update the state
                        old_state->copy(next_state);
                        if (old_state->closed) {
                            // reopen closed state
                            old_state->closed=false;
                            old_state->open_list_handle=open_list.push(old_state);
                        } else {
                            // update open state
                            open_list.increase(old_state->open_list_handle);
                        }
                    }
                    delete next_state;
                }
            }
        }
    }

    void route() {

        // TODO: maybe we can use flow optimization to route agents

        std::vector<std::pair<int,int> > agent_ids;
        for (auto & agent_controller: agent_controllers) {
            if (agent_controller.curr_region==-1) {
                // re-route for this agent
                int start_pos=env.curr_states[agent_controller.id].location;
                int start_orient=env.curr_states[agent_controller.id].orientation;
                int goal_pos=env.goal_locations[agent_controller.id][0].first;
                agent_ids.emplace_back(HT->get(start_pos,start_orient,goal_pos),agent_controller.id);
            }
        }

        std::cout<<"agent_ids.size()="<<agent_ids.size()<<std::endl;

        // TODO: sort based on some metric or simply shuffle
        std::sort(agent_ids.begin(), agent_ids.end());

        for (auto & pair: agent_ids) {
            int agent_id=pair.second;
            int start=env.curr_states[agent_id].location;
            int goal=env.goal_locations[agent_id][0].first;
            findPath(agent_id, start, goal);
        }

    }

    void update(int agent_id, int prev, int curr, bool arrived) {
        // int prev_region=get_region_index(prev);
        int curr_region=get_region_index(curr);

        int next_region_to_go_idx=agent_controllers[agent_id].next_region_to_go_idx;
        int next_region_to_go=agent_controllers[agent_id].region_path[next_region_to_go_idx];

        if (curr_region!=next_region_to_go) {
            // agent deviated from its region path
            // we re-route it now
            // TODO: or we should ask it to go back to the previous region?
            reset_agent_controller(agent_id);
        } else {
            agent_controllers[agent_id].next_region_to_go_idx++;
        }

        if (arrived) {
            // agent arrives at its goal
            // we also re-route it
            reset_agent_controller(agent_id);
        } 

        agent_controllers[agent_id].curr_region=curr_region;

    }

    void reset_agent_controller(int agent_id) {
        agent_controllers[agent_id].reset();
        for (auto & region_id: agent_controllers[agent_id].region_path) {
            regions[region_id]->congestion--;
        }
    }

};  

}