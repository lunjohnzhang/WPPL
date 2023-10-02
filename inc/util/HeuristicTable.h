#pragma once
#include "common.h"
#include "SharedEnv.h"
#include <omp.h>
#include <chrono>
#include "util/CompetitionActionModel.h"
#include "boost/filesystem.hpp"
#include <boost/iostreams/filtering_streambuf.hpp>
#include <boost/iostreams/copy.hpp>
#include <boost/iostreams/filter/gzip.hpp>
#include "util/Dev.h"
#include "util/Timer.h"
#include "util/MyLogger.h"
#include "boost/format.hpp"
#include "util/SearchForHeuristics/SpatialSearch.h"
#include "util/SearchForHeuristics/SpatialSearchBoost.h"
// #include "bshoshany/BS_thread_pool.hpp"


#define MAX_HEURISTIC INT_MAX/16

class HeuristicTable {
public:

    const SharedEnvironment & env;
    CompetitionActionModelWithRotate action_model;
    // loc1, loc2
    unsigned short * main_heuristics;
    // loc1, loc2, orient1, orient2
    char * sub_heuristics;
    int * empty_locs;
    int * loc_idxs; 
    int n_orientations=4;
    size_t loc_size=0;
    size_t state_size;
    bool consider_rotation=true;

    HeuristicTable(SharedEnvironment * _env, bool consider_rotation=true):env(*_env),action_model(_env),consider_rotation(consider_rotation){
        if (consider_rotation) {
            n_orientations=4;
        } else {
            n_orientations=1;
        }

        loc_size=0;
        for (int i=0;i<env.map.size();++i) {
            if (!env.map[i]) {
                ++loc_size;
            }
        }

        empty_locs = new int[loc_size];
        loc_idxs = new int[env.map.size()];

        g_logger.debug("number of empty locations: {}", loc_size);
        std::fill(loc_idxs,loc_idxs+env.map.size(),-1);

        int loc_idx=0;
        for (int loc=0;loc<env.map.size();++loc) {
            if (!env.map[loc]) {
                empty_locs[loc_idx]=loc;
                loc_idxs[loc]=loc_idx;
                ++loc_idx;
            }
        }
        ONLYDEV(assert(loc_idx==loc_size);)

        state_size = loc_size*n_orientations;
        main_heuristics = new unsigned short[loc_size*loc_size];
        if (consider_rotation)
            sub_heuristics = new char[state_size*state_size];
    };

    ~HeuristicTable() {
        delete [] empty_locs;
        delete [] loc_idxs;
        delete [] main_heuristics;
        if (consider_rotation)
            delete [] sub_heuristics;
    }

    // weights is an array of [loc_size*n_orientations]
    void compute_heuristics(const std::vector<int> & weights){
        g_logger.debug("[start] Compute heuristics.");
        g_timer.record_p("heu/compute_start");

        int n_threads=omp_get_max_threads();
        // BS::thread_pool pool(n_threads);

        // int n_threads=pool.get_thread_count();
        cout<<"number of threads used for heuristic computation: "<<n_threads<<endl;
        unsigned short * values = new unsigned short[n_threads*n_orientations*state_size];
        RIVERS::SPATIAL::SpatialAStar ** planners= new RIVERS::SPATIAL::SpatialAStar* [n_threads];
        for (int i=0;i<n_threads;++i) {
            planners[i]=new RIVERS::SPATIAL::SpatialAStar(env,n_orientations,weights);
        }

        cerr<<"created"<<endl;

        int ctr=0;
        int step=100;
        auto start = std::chrono::steady_clock::now();

        // std::queue<int> tasks;
        // for (int loc_idx=0;loc_idx<loc_size;++loc_idx) {
        //     tasks.push(loc_idx);
        // }

        // std::vector<std::future<void>> futures(n_threads);
        // std::queue<int> empty_slots;
        // for (int i=0;i<n_threads;++i) {
        //     empty_slots.push(i);
        // }

        // while (!tasks.empty()) {
        //     int loc_idx=tasks.front();
        //     tasks.pop();

        //     while (empty_slots.size()==0) {
        //         std::this_thread::sleep_for(std::chrono::milliseconds(10));
        //         for (int i=0;i<futures.size();++i) {
        //             if (futures[i].wait_for(std::chrono::milliseconds(0))==std::future_status::ready) {
        //                 // futures[i].get();
        //                 empty_slots.push(i);
        //                 ++ctr;
        //             }
        //         }
        //     }

        //     int thread_id=empty_slots.front();
        //     empty_slots.pop();
        //     futures[thread_id]=pool.submit(
        //         &HeuristicTable::_compute_heuristics,
        //         this,
        //         loc_idx,
        //         values+thread_id*n_orientations*state_size,
        //         planners[thread_id]
        //     );

        //     if (ctr%step==0){
        //         auto end = std::chrono::steady_clock::now();
        //         double elapse=std::chrono::duration<double>(end-start).count();
        //         double estimated_remain=elapse/ctr*(loc_size-ctr);
        //         cout<<ctr<<"/"<<loc_size<<" completed in "<<elapse<<"s. estimated time to finish all: "<<estimated_remain<<"s.  estimated total time: "<<(estimated_remain+elapse)<<"s."<<endl;
        //     }
        // }

        // while (empty_slots.size()!=n_threads) {
        //     std::this_thread::sleep_for(std::chrono::milliseconds(10));
        //     for (int i=0;i<futures.size();++i) {
        //         if (futures[i].wait_for(std::chrono::milliseconds(0))==std::future_status::ready) {
        //             // futures[i].get();
        //             empty_slots.push(i);
        //             ++ctr;
        //         }
        //     }

        //     if (ctr%step==0){
        //         auto end = std::chrono::steady_clock::now();
        //         double elapse=std::chrono::duration<double>(end-start).count();
        //         double estimated_remain=elapse/ctr*(loc_size-ctr);
        //         cout<<ctr<<"/"<<loc_size<<" completed in "<<elapse<<"s. estimated time to finish all: "<<estimated_remain<<"s.  estimated total time: "<<(estimated_remain+elapse)<<"s."<<endl;
        //     }
        // }

        #pragma omp parallel for schedule(dynamic,1)
        for (int loc_idx=0;loc_idx<loc_size;++loc_idx)
		{
            int thread_id=omp_get_thread_num();

            int s_idx=thread_id*n_orientations*state_size;
 
            _compute_heuristics(loc_idx,values+s_idx,planners[thread_id]);


            #pragma omp critical
            {
                ++ctr;
                if (ctr%step==0){
                    auto end = std::chrono::steady_clock::now();
                    double elapse=std::chrono::duration<double>(end-start).count();
                    double estimated_remain=elapse/ctr*(loc_size-ctr);
                    cout<<ctr<<"/"<<loc_size<<" completed in "<<elapse<<"s. estimated time to finish all: "<<estimated_remain<<"s.  estimated total time: "<<(estimated_remain+elapse)<<"s."<<endl;
                }
            }

            // ONLYDEV(if (empty_locs[loc_idx]==2) {
            //     dump_main_heuristics(
            //         2,
            //         "analysis/heuristics/debug"
            //     );
            // })
		}

        delete [] values;
        for (int i=0;i<n_threads;++i) {
            delete planners[i];
        }
        delete planners;

        g_timer.record_d("heu/compute_start","heu/compute_end","heu/compute");

        g_logger.debug("[end] Compute heuristics. (duration: {:.3f})", g_timer.get_d("heu/compute"));
    }

    void _compute_heuristics(
        int start_loc_idx,
        unsigned short * values,
        RIVERS::SPATIAL::SpatialAStar * planner
    ) {
        planner->reset();
        int start_loc=empty_locs[start_loc_idx];
        if (!consider_rotation){
            planner->search_for_all(start_loc,-1);
          
            // for (auto & state: planner->all_states) {
            //     int loc_idx=loc_idxs[state->pos];
            //     size_t main_idx=start_loc_idx*loc_size+loc_idx;
            //     main_heuristics[main_idx]=state->g;
            // }
          
            for (int loc_idx=0;loc_idx<loc_size;++loc_idx) {
                int loc=empty_locs[loc_idx];
                RIVERS::SPATIAL::State * state=planner->all_states+loc;
                if (loc!=state->pos) {
                    std::cerr<<"loc: "<<loc<<" state->pos: "<<state->pos<<endl;
                    exit(-1);
                }
                // if (start_loc==100)
                //     std::cerr<<start_loc<<" "<<state->pos<<" "<<state->g<<endl;
                int cost=state->g;
                if (cost==-1) {
                    cost=USHRT_MAX;
                }
                size_t main_idx=start_loc_idx*loc_size+loc_idx;
                main_heuristics[main_idx]=cost;
            }
            // if (start_loc==0) exit(-1);
        } else {
            g_logger.error("compute_heuristics with both orient and weight is not supported now!");
            exit(-1);
        }
    }

    void dump_main_heuristics(int start_loc, string file_path_prefix) {
        int start_loc_idx=loc_idxs[start_loc];
        if (start_loc_idx==-1) {
            std::cerr<<"error: start_loc_idx==-1"<<endl;
            exit(-1);
        }
        string file_path=file_path_prefix+"_"+std::to_string(start_loc)+".main_heuristics";
        std::ofstream fout(file_path);
        for (int i=0;i<env.rows;++i) {
            for (int j=0;j<env.cols;++j) {
                int target_loc=i*env.cols+j;
                int target_loc_idx=loc_idxs[target_loc];
                int h=USHRT_MAX;
                if (target_loc_idx!=-1){
                    h=main_heuristics[start_loc_idx*loc_size+target_loc_idx];
                }
                fout<<h;
                if (j!=env.cols-1) {
                    fout<<",";
                }
            }
            fout<<endl;
        }
    }

    void compute_heuristics(){
        g_logger.debug("[start] Compute heuristics.");
        g_timer.record_p("heu/compute_start");

        int n_threads=omp_get_max_threads();
        cout<<"number of threads used for heuristic computation: "<<n_threads<<endl;
        unsigned short * values = new unsigned short[n_threads*n_orientations*state_size];
        bool * visited = new bool[n_threads*n_orientations*state_size];
        State * queues = new State[n_threads*n_orientations*state_size];

        cerr<<"created"<<endl;


        int ctr=0;
        int step=100;
        auto start = std::chrono::steady_clock::now();
        #pragma omp parallel for
        for (int loc_idx=0;loc_idx<loc_size;++loc_idx)
		{
            int thread_id=omp_get_thread_num();

            int s_idx=thread_id*n_orientations*state_size;
            compute_heuristics(loc_idx,values+s_idx,visited+s_idx,queues+s_idx);
            
            #pragma omp critical
            {
                ++ctr;
                if (ctr%step==0){
                    auto end = std::chrono::steady_clock::now();
                    double elapse=std::chrono::duration<double>(end-start).count();
                    double estimated_remain=elapse/ctr*(loc_size-ctr);
                    cout<<ctr<<"/"<<loc_size<<" completed in "<<elapse<<"s. estimated time to finish all: "<<estimated_remain<<"s.  estimated total time: "<<(estimated_remain+elapse)<<"s."<<endl;
                }
            }
		}

        delete [] values;
        delete [] visited;
        delete [] queues;

        g_timer.record_d("heu/compute_start","heu/compute_end","heu/compute");

        g_logger.debug("[end] Compute heuristics. (duration: {:.3f})", g_timer.get_d("heu/compute"));
    }

    inline void _push(State * queue, State &s, int & e_idx) {
        queue[e_idx]=s;
        e_idx+=1;
    }

    inline State _pop(State * queue, int & s_idx) {
        State & s = queue[s_idx];
        s_idx+=1;
        return s;
    }

    inline bool _empty(int s_idx, int e_idx) {
        return s_idx==e_idx;
    }

    void compute_heuristics(
        int start_loc_idx,
        unsigned short * values, 
        bool * visited,
        State * queue
    ) {
        std::fill(values,values+n_orientations*state_size,USHRT_MAX);
        std::fill(visited,visited+n_orientations*state_size,false);

        int start_loc=empty_locs[start_loc_idx];

        for (int start_orient=0;start_orient<n_orientations;++start_orient) {
            // this are indexs used for queues.
            int s_idx=start_orient*state_size;
            int e_idx=start_orient*state_size;

            // start
            State state(start_loc,0,start_orient);
            ONLYDEV(assert(loc_idxs[state.location]>=0);)
            // start_orient,loc,orient
            size_t idx=start_orient*state_size+loc_idxs[state.location]*n_orientations+state.orientation;
            // because this is uniform-cost bfs, we know the first time is the best time
            values[idx]=state.timestep;
            visited[idx]=true;
            _push(queue,state,e_idx);

            int ctr=0;
            while (!_empty(s_idx,e_idx)) {
                State state=_pop(queue,s_idx);
                ++ctr;
                int c=0;

                vector<State> neighbors;
#ifndef NO_ROT
                if (consider_rotation){
                    neighbors=action_model.get_state_neighbors(state,false);
                } else {
                    neighbors=action_model.get_loc_neighbors(state,false);
                }
#else
                if (consider_rotation){
                    cerr<<"no valid to set on consider_rotation when compiled with NO_ROT"<<endl;
                    exit(-1);
                } else {
                    neighbors=action_model.get_loc_neighbors(state,false);
                }
#endif

                for (auto & next: neighbors){
                    ++c;
                    ONLYDEV(assert(loc_idxs[state.location]>=0);)
                    size_t idx=start_orient*state_size+loc_idxs[next.location]*n_orientations+next.orientation;
                    if (!visited[idx]) {
                        values[idx]=next.timestep;
                        assert(next.timestep==state.timestep+1 && next.timestep<USHRT_MAX);
                        visited[idx]=true;
                        _push(queue,next,e_idx);
                    }
                }
            }
        }


        if (consider_rotation) {
            for (int loc_idx=0;loc_idx<loc_size;++loc_idx) {
                unsigned short min_val=USHRT_MAX;
                for (int start_orient=0;start_orient<n_orientations;++start_orient) {
                    for (int orient=0;orient<n_orientations;++orient) {
                        size_t value_idx=start_orient*state_size+loc_idx*n_orientations+orient;
                        // cerr<<"helo"<<values[value_idx]<<endl;
                        if (values[value_idx]<min_val) {
                            min_val = values[value_idx];
                        }
                    }
                }
                // cerr<<start_loc_idx<<" "<<loc_idx<<" "<<min_val<<endl;
                size_t main_idx=start_loc_idx*loc_size+loc_idx;
                main_heuristics[main_idx]=min_val;
                for (int start_orient=0;start_orient<n_orientations;++start_orient) {
                    for (int orient=0;orient<n_orientations;++orient) {
                        size_t value_idx=start_orient*state_size+loc_idx*n_orientations+orient;
                        size_t sub_idx=((start_loc_idx*loc_size+loc_idx)*n_orientations+start_orient)*n_orientations+orient;
                        sub_heuristics[sub_idx]=char(values[value_idx]-min_val);
                    }
                }
            }
        } else {
            for (int loc_idx=0;loc_idx<loc_size;++loc_idx) {
                size_t main_idx=start_loc_idx*loc_size+loc_idx;
                main_heuristics[main_idx]=values[loc_idx];
            }
        }

    }
    
    // TODO(hj) add check
    inline int get(int loc1, int loc2) {
        int loc_idx1=loc_idxs[loc1];
        int loc_idx2=loc_idxs[loc2];

        if (loc_idx1==-1 || loc_idx2==-1) {
            return MAX_HEURISTIC;
        }

        size_t idx=loc_idx1*loc_size+loc_idx2;
        return main_heuristics[idx];
    }

    inline int get(int loc1, int orient1, int loc2) {
        if (!consider_rotation) {
            cerr<<"no valid to use this func if not consider rotation"<<endl;
            exit(-1);
        }

        int loc_idx1=loc_idxs[loc1];
        int loc_idx2=loc_idxs[loc2];

        if (loc_idx1==-1 || loc_idx2==-1) {
            return MAX_HEURISTIC;
        }

        size_t idx=((loc_idx1*loc_size+loc_idx2)*n_orientations+orient1)*n_orientations;
        char min_v=CHAR_MAX;
        for (int i=0;i<n_orientations;++i) {
            if (sub_heuristics[idx+i]<min_v) {
                min_v=sub_heuristics[idx+i];
            }
        }
        return min_v+main_heuristics[loc_idx1*loc_size+loc_idx2];
    } 

    inline int get(int loc1, int orient1, int loc2, int orient2) {
        if (!consider_rotation) {
            cerr<<"no valid to use this func if not consider rotation"<<endl;
            exit(-1);
        }
        int loc_idx1=loc_idxs[loc1];
        int loc_idx2=loc_idxs[loc2];

        if (loc_idx1==-1 || loc_idx2==-1) {
            return MAX_HEURISTIC;
        }

        size_t idx=((loc_idx1*loc_size+loc_idx2)*n_orientations+orient1)*n_orientations+orient2;
        return sub_heuristics[idx]+main_heuristics[loc_idx1*loc_size+loc_idx2];
    }


    void preprocess() {

        string fname=env.map_name.substr(0,env.map_name.size()-4);
        string folder=env.file_storage_path;
        if (folder[folder.size()-1]!=boost::filesystem::path::preferred_separator){
            folder+=boost::filesystem::path::preferred_separator;
        }
        string fpath;
        if (consider_rotation) {
            fpath=folder+fname+"_heuristics_v2.gz";
        } else {
            fpath=folder+fname+"_heuristics_no_rotation_v2.gz";
        }

        if (boost::filesystem::exists(fpath)) {
            load(fpath);
        } else {
            compute_heuristics();
            save(fpath);
        }

    }

    void preprocess(const std::vector<int> & weights, string suffix="") {

        string fname=env.map_name.substr(0,env.map_name.size()-4);
        string folder=env.file_storage_path;
        if (folder[folder.size()-1]!=boost::filesystem::path::preferred_separator){
            folder+=boost::filesystem::path::preferred_separator;
        }
        string fpath;
        if (consider_rotation) {
            fpath=folder+fname+"_weighted_heuristics_v2_"+suffix+".gz";
        } else {
            fpath=folder+fname+"_weighted_heuristics_no_rotation_v2_"+suffix+".gz";
        }

        if (boost::filesystem::exists(fpath)) {
            load(fpath);
        } else {
            compute_heuristics(weights);
            save(fpath);
        }
    }

    void save(const string & fpath) {
        g_logger.debug("[start] Save heuristics to {}.", fpath);
        g_timer.record_p("heu/save_start");

        std::ofstream fout;
        fout.open(fpath,std::ios::binary|std::ios::out);

        boost::iostreams::filtering_streambuf<boost::iostreams::output> outbuf;
        outbuf.push(boost::iostreams::zlib_compressor());
        outbuf.push(fout);
        
        std::ostream out(&outbuf);

        // save loc size
        out.write((char *)&loc_size,sizeof(int));

        // save empty locs
        out.write((char*)empty_locs,sizeof(int)*loc_size);

        // save main heuristics
        out.write((char *)main_heuristics,sizeof(unsigned short)*loc_size*loc_size);

        // save sub heuristics
        if (consider_rotation)
            out.write((char *)sub_heuristics,sizeof(char)*state_size*state_size);

        boost::iostreams::close(outbuf);
        fout.close();
        
        g_timer.record_d("heu/save_start","heu/save_end","heu/save");

        g_logger.debug("[end] Save heuristics to {}. (duration: {:.3f})", fpath, g_timer.get_d("heu/save"));
    }

    void load(const string & fpath) {
        g_logger.debug("[start] load heuristics from {}.",fpath);
        g_timer.record_p("heu/load_start");
        std::ifstream fin;
        fin.open(fpath,std::ios::binary|std::ios::in);

        boost::iostreams::filtering_streambuf<boost::iostreams::input> inbuf;
        inbuf.push(boost::iostreams::zlib_decompressor());
        inbuf.push(fin);

        std::istream in(&inbuf);

        // load loc size
        int _loc_size;
        in.read((char *)&_loc_size,sizeof(int));

        // check loc size
        if (_loc_size!=loc_size) {
            cerr<<"the sizes of empty locations don't match!"<<endl;
            exit(-1);
        }

        // load empty locs
        int * _empty_locs=new int[loc_size];
        in.read((char *)_empty_locs,sizeof(int)*loc_size);

        // check empty locs
        for (auto i=0;i<loc_size;++i) {
            if (_empty_locs[i]!=empty_locs[i]) {
                cerr<<"the empty locations don't match!"<<endl;
                exit(-1);
            }
        }

        // load main heurisitcs
        in.read((char *)main_heuristics,sizeof(unsigned short)*loc_size*loc_size);
        
        // load sub heuristics
        if (consider_rotation)
            in.read((char *)sub_heuristics,sizeof(char)*state_size*state_size);

        g_timer.record_d("heu/load_start","heu/load_end","heu/load");

        g_logger.debug("[end] load heuristics from {}. (duration: {:.3f})",fpath,g_timer.get_d("heu/load"));
    }
};